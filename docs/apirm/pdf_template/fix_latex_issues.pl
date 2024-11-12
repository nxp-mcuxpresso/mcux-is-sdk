#!C:\Perl\bin\perl.exe
#print Processing Latex Files....
open STDOUT, ">", "output.txt" or die "$0: open: $!";
open STDERR, ">&STDOUT"        or die "$0: dup: $!";

use File::Copy;
use utf8;
binmode *STDOUT, ':encoding(utf8)';

$tex_folder="latex";


if ($#ARGV >= 0) {
print "We have an argument to parse";
$tex_folder = $ARGV[0];
}

undef @fileArray;
undef @lblArr
&readRefManTex();
&processLaTexDirectory();
&fixlatexhierarchy();

# 1)Parse "refman.tex" and look for "\chapter{Module Documentation}"
# 2) Copy those references into the array until you encounter the next "\chapter"
# 3) For those references that you copied, delete the "\input" and append .tex
# 4) Parse that list and run that modification script on those references.

sub readRefManTex {
	print "Inside readRefManTex";
	$refManLoc = $tex_folder."\\refman.tex";
	print "$refManLoc\n";
	open(READF,"$refManLoc") || die "File not found\n";
	open(WRITF,">tmp-refman") || die "File not found\n";
	print "Opened $refManLoc for processing\n";
	my $insideChapter = 0;
	my $startReading = 0;
	my $linenumber = 0;
	my $srchtxt = "Module Documentation";
	while (my $line = readline(READF)) {
		$linenumber++;
		#Set the boolean variable to when processing should start.
		if ($line =~ m{%===== C O N T E N T S =====}) {
			$startReading = 1;
		}
		#Set the boolean variable to when processing should end.
		if ($line =~ m{% Backmatter}) {
			$startReading = 0;
		}
		#We are in a section that should be processed.    
		if ($startReading) {
			if ($line =~ m/\{$srchtxt\}/) {
				binmode *STDOUT, ':encoding(utf8)';			
				$line =~ s/\\chapter{$srchtxt\}//g;
				#print "Removed Module Documentation chapter\n";
				$insideChapter++;
			}
			if ($line =~ m{\\chapter}) {
				$line =~ s/chapter/part/g;
				$line = $line . "\\nopagebreak";
				#print "MODIFIED Line : $line\n";
				$insideChapter++;
			}

			if ($line =~ m/\\include\{/) {
				$line =~ s/\\include\{/\\input\{/g;
				$insideChapter++;
			}	
			
			if ($line =~ m/\\label/) {
				#print "LABEL : $&\n";
				$lbl_line = $line;
				$lbl_line =~ s/\\label\{//g;
				@tempName = split "}", $lbl_line;
				push(@lblArr,$tempName[0]);
			}
		
		}	 
		print WRITF $line;
	}	
	close READF;
	close WRITF;
	print "Closed $refManLoc for processing\n";
	if($insideChapter>0){
		system "move tmp-refman $refManLoc";
		#print  "mv -f tmp-refman $refManLoc\n";
	}
	else {
		print "Nothing to change in file $refManLoc\n";
	}
}




sub processLaTexDirectory {
use utf8;
use charnames ':full'; /\N{EM DASH}/;
	foreach $fname (@lblArr){
		$latex_file = $tex_folder ."\\". $fname;
		#print "$latex_file\n";
		open (fp,"$latex_file.tex") or return "cannot open $latex_file.tex";
		my @texlines = <fp>;
		close fp;
		open (newfp,">$tex_folder\\tempfile");
		$x=0;
		foreach $texln(@texlines)
		{

			if ($texln =~ m/\\section/)
			{
				$texln =~ s/section/nopagebreak \\chapter/g;
				#print "\nchanged :: level$subsublvl,$texln\n";
				$x++;
			}			
			elsif ($texln =~ m/\\subsection/)
			{
				$texln =~ s/subsection/nopagebreak \\section/g;
				#print "\nchanged :: level$sublvl,$texln\n";
				$x++;	
			}
			elsif ($texln =~ m/\\subsubsection/)
			{
				$texln =~ s/subsubsection/nopagebreak \\subsection/g;
				#print "\nchanged :: level$currlevel,$texln\n";
				$x++;
			}
			elsif ($texln =~ m/\\paragraph/)
			{
				$texln =~ s/paragraph/nopagebreak \\subsubsection/g;
				#print "\nchanged :: level$currlevel,$texln\n";
				$x++;
			}
			elsif ($texln =~ m/\\subparagraph/)
			{
				$texln =~ s/subparagraph/nopagebreak \\paragraph/g;
				#print "\nchanged :: level$currlevel,$texln\n";
				$x++;
			}
		elsif ($texln =~ m/\\rowcolor{lightgray\}/)
		{
			$texln =~ s/rowcolor\{lightgray\}/ /g;
			#print "Instance of rowcolor\{lightgray\} changed\n";
			$x++;	
		}							
		print newfp $texln;
		}	
		close newfp;
		if($x>0){
			print  "\nchanged File : $latex_file.tex\n";
			system "move $tex_folder\\tempfile $latex_file.tex";
			#print  "mv -f $tex_folder\\tempfile $latex_file.tex\n";
		}
		else {
			print "Nothing to change in file $latex_file.tex\n";
		}
}
print "Completed Changing files\n";
}

sub fixlatexhierarchy()
{
$html_folder="html";
$tex_folder="latex";
$i=0;
@fname = 'modules';

for ($i=0;$i<=7;$i++)
{
	@newarr = ();
	foreach $flname(@fname){
	#	print "\n$flname,level $i\n";
		if($flname ne "modules"){ changetexfile($flname,$i)};
		open(fd,"$html_folder\\$flname.js") or print "$html_folder\\$flname doesnot exist" ;
		@lines = <fd>;
		close fd;
	        foreach $line(@lines)
		{
			if ($line =~ m/group__.*\.html"/g)
			{
				$curr_line = $&;
				$curr_line =~s/.html"//g;
				push (@newarr,$curr_line);
			}			
		}	

	}
	if (scalar(@newarr)==0)
	{
		print "no more files in list \n";
		last;
	}
	@fname=@newarr;
}
print "\nCompleted changing the files\n";
exit;
}

sub changetexfile()
{
	$texfilename = $_[0];
	my $currlevel = ($_[1]<=7) ? $_[1] : 7;
	my $subparvl = $parvl = $sublvl = $subsublvl = 0;
	my $en_dash = "\x{0096}";
	@vals = ("noval","part","chapter","section","subsection","subsubsection","paragraph","subparagraph");
	print "changing $texfilename starting with level $currlevel\n";
 
	open (fp,"$tex_folder\\$texfilename.tex") or return "cannot open $tex_folder\\$texfilename.tex";
	my @texlines = <fp>;
	close fp;
	open (newfp,">$tex_folder\\tempfile");
	$x=0;
	foreach $texln(@texlines)
	{
		if ($texln =~ m/subparagraph/)
		{
			$subparvl=(($currlevel+4) <=7) ? ($currlevel+4) : 7;
	#		$subparvl=$currlevel+4;
			#print "level5,$texln";
			$texln =~ s/\bsubparagraph\b/$vals[$subparvl]/g;
			#print "\nchanged :: level$subparvl,$texln\n";
			$x++;
		}			
		elsif ($texln =~ m/paragraph/)
		{
			$parvl=(($currlevel+3) <=7) ? ($currlevel+3) : 7;
	#		$parvl=$currlevel+3;
			#print "level4,$texln";
			$texln =~ s/\bparagraph\b/$vals[$parvl]/g;
			#print "\nchanged :: level$parvl,$texln\n";
			$x++;
		}			
		elsif ($texln =~ m/subsubsection/)
		{
			$subsublvl=(($currlevel+2) <=7) ? ($currlevel+2) : 7;
	#		$subsublvl=$currlevel+2;
			#print "level3,$texln";
			$texln =~ s/\bsubsubsection\b/$vals[$subsublvl]/g;
			#print "\nchanged :: level$subsublvl,$texln\n";
			$x++;
		}		
		elsif ($texln =~ m/subsection/)
		{
			$sublvl=(($currlevel+1)<=7) ? ($currlevel+1) : 7;
	#		$sublvl=$currlevel+1;
			#print "level2,$texln";
			$texln =~ s/\bsubsection\b/$vals[$sublvl]/g;
			#print "\nchanged :: level$sublvl,$texln\n";
			$x++;	
		}
		elsif ($texln =~ m/$en_dash/)
		{
			binmode *STDOUT, ':encoding(utf8)';
			$texln =~ s/$en_dash/-/g;
			#print "\nchanged :: level$sublvl,$texln\n";
			$x++;	
		}
		elsif ($texln =~ m/\bsection\b/)
		{
			#print "level$currlevel,$texln";
			$texln =~ s/section/$vals[$currlevel]/g;
			#$texln =~ s/section/level$currlevel/g;
			if($vals[$currlevel] eq "part") 
			{
				$texln = $texln . "\\nopagebreak";
			}
			#print "\nchanged :: level$currlevel,$texln\n";
			$x++;
		}
		elsif ($texln =~ /\{lightgray\}/)
		{
			$texln =~ s/\{lightgray\}/\{\}/g;
			#print "Instance of rowcolor\{lightgray\} changed\n";
			$x++;	
		}			

	print newfp $texln;
	}	
	close newfp;
	if($x>0){
		#system "mv modlatex\\tempfile latex\\$texfilename";
		system "move $tex_folder\\tempfile $tex_folder\\$texfilename.tex";
		#print  "mv -f $tex_folder\\tempfile $tex_folder\\$texfilename.tex\n";
	}
	else {
		print "Nothing to change in file $tex_folder\\$texfilename.tex\n";
	}
}

