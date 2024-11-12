#!C:\Perl\bin\perl.exe

use File::Copy;
use Cwd;


&processLaTexDirectory(cwd());


#Process the Latex Directory
sub processLaTexDirectory {
   my $directory = $_[0];
   copy ('doxygen.sty', $directory.'\doxygen.sty');

   opendir(D, "$directory") || die "Can't opedir $directory: $!\n";
   my @list = readdir(D);
   closedir(D);

   print "directory name is $directory\n";
   foreach my $f (@list) {
     my $fileType = substr ($f,-4);
     if ($fileType =~ /.tex/) {
        &processTexFiles($f,$directory);
     }
   }
}


#Subroutine that will process the tex file
#   Input to processTexFiles:
#   Argument 0:  File Name
sub processTexFiles {
    my $filename = $_[0];
    my $directory = $_[1];
    my $tmpFileName = $filename.".tmp";
    open(READ,"<",$directory.'/'.$filename);
    open(WRITE,">",$directory.'/'.$tmpFileName);
    #Loop through the file
    my $previousLineStatus = 0;
    my $previousLine = '';
    my $lineNumber = 1;
    while(my $line = readline(READ)) {
       my $status = &specialTexLine($line);
       &processTexLine($previousLineStatus,$previousLine,$status,$line);
       $previousLineStatus = $status;
       $previousLine = $line;
       $lineNumber++;
    }
    close WRITE;
    close READ;
    move $directory.'/'.$tmpFileName, $directory.'/'.$filename;
}

# Subroutine:
#   Name:  specialTexLine
#      -  Looks at the current line to see if it contains the 
#   Arguments:
#      texLine :: String ::: Contains the current text line
#   Returns
#      Integer that contains what the string contains
#      0 ::: String is a regular string ... No processing required
#      1 ::: String contains \begin{TabularC}
#      2 ::: String begins with \rowcolor{lightgray}
#      3 ::: String contains \end{TabularC}
sub specialTexLine {
    my $texLine = $_[0];
    if ($texLine =~ /\\begin\{TabularC\}/) {
        1;
    } elsif ($texLine =~ /^\\rowcolor\{lightgray\}/) {
        2;
    } elsif ($texLine =~ /^\\end\{TabularC\}/) {
        3;
    } else {
        0;
    }
}

# Subroutine:
# Name:  processTexLine
#    -- This will process the text Line
#    -- After processing the line, the line will be written to the WRITE File handler
# Arguments:
#    previousLineStatus ::: Integer ::: Integer value of the previous Line --- from SpecialTexLine
#    previousLine ::: String ::: String of the previous Line
#    currentLineStatus ::: Integer ::: Integer value of the current Line --- from SpecialTexLine
#    currentLine ::: String ::: String of the current Line
# Returns
#    Nothing
sub processTexLine {
    my $previousLineStatus = $_[0];
    my $previousLine = $_[1];
    my $currentLineStatus = $_[2];
    my $currentLine = $_[3];
	if (($previousLineStatus == 2) && ($currentLineStatus != 2)) {
	   #Write the previousLine after it has been parsed...Add in \\endhead at the end of the line
	   #Write the currentLine since it does not have to be parsed and reworked
	   &writeRowColorLine($previousLine,1);
	   print WRITE $currentLine;
	} elsif ($previousLineStatus == 2) {
	   #Write the previousLine and not add \\endhead to it since it is not the end of the head rows
	   &writeRowColorLine($previousLine,0);
	} elsif (($currentLineStatus == 2) && ($previousLineStatus != 2)) {
	   #Don't do anything here...		
	} else {
	   #Write the current Line
	   print WRITE $currentLine;
	}
}

# Subroutine
# Name :  writeRowColorLine
#  -- This will write out a line that starts \rowcolor{lightgray} with the right setup
# Arguments :
#  -- textLine ::: String ::: line that needs to be parsed and rewritten ... The line will look like
#\rowcolor{lightgray}{\bf Column Name }&{\bf Type }&{\bf Constraints }&{\bf Description  }\\\cline{1-4}
#  -- lineOption ::: Integer ::: 
#     0 ::: do not add \endhead to the line
#     1 ::: add \endhead to the line
# Returns:
#  Nothing
sub writeRowColorLine {
   my $textLine = $_[0];
   my $lineOption = $_[1];
   #\rowcolor{lightgray}{\bf Column Name }&{\bf Type }&{\bf Constraints }&{\bf Description  }\\\cline{1-4}
   #Remove \rowcolor{lightgray} from the start
   my $tmpStr = $textLine;
   $tmpStr =~ s/^\\rowcolor\{lightgray\}//;
   #String now looks like
   #{\bf Column Name }&{\bf Type }&{\bf Constraints }&{\bf Description  }\\\cline{1-4}
   #Get cline information
   my @splitFirst = split('cline',$tmpStr);
   my $clineInfo = $splitFirst[1];
   my $columnInfo = substr($splitFirst[0],0,-3);
   my @columns = split('&',$columnInfo);
   my $tmpFnl = '';
   my $centerStr = '\\multicolumn{1}{|c|}';
   my $colSize = @columns;
   my $colLCV = 1;
   for my $colStr (@columns) {
      $tmpFnl .= $centerStr . $colStr;
      if ($colLCV != $colSize) {
         $tmpFnl .= '&';
      } else {
         $tmpFnl .= '\\ \hline'."\n";
      }
      $colLCV++;
   }
   if ($lineOption == 1) {
      $tmpFnl .= '\endhead',"\n";
   } else {
      $tmpFnl .= '\\'."\n";
   }
   $tmpFnl .= '\cline' . $clineInfo;
   print WRITE $tmpFnl;
}
