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
	while (glob "*.dep") {
		update($old, $new, $_);
	}
	while (glob "*.ewp") {
		update($old, $new, $_);
	}
	while (glob "*.ewt") {
		update($old, $new, $_);
	}
}
updatefiles("SDK_2.1_FRDM-K22F", "SDK_2.2_FRDM-K22F");
updatefiles("issdk_1.0", "issdk_1.5");

