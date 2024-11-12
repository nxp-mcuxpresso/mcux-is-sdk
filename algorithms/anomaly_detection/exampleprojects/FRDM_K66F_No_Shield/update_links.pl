use File::Copy;

sub update {
	my ($old, $new, $fn) = @_;
	copy($fn, $fn.".orig");
	open(infile, "<$fn.orig");
	open(outfile, ">$fn.updated");
	while (<infile>) {
		s/$old/$new/g;
		printf outfile "%s", $_;
	}
	close(infile);
	close(outfile);
	move("$fn.updated", "$fn");
}
sub updatefiles {
	my ($old, $new) = @_;
	while (glob "*/*.dep") {
		update($old, $new, $_);
	}
	while (glob "*/*.ewp") {
		update($old, $new, $_);
	}
	while (glob "*/*.ewt") {
		update($old, $new, $_);
	}
}
updatefiles("SDK_2.0_FRDM-K66F", "SDK_2.1_FRDM-K66F");
updatefiles("freertos_8.2.3", "freertos_9.0.0");

