#!/usr/bin/perl

use Chart::Plot;
use Math::Trig;
use Math::Round qw(:all); # For nearest()

sub help {
    print("usage: sweep_convert.pl <filename>\n");
}

if ($#ARGV != 0) {
    help();
    exit;
}

$filename = $ARGV[0];

print("Processing: $filename\n");

$data = `cat $filename`;
@lines = split("\n", $data);

@xpts;
@ypts;
foreach $line (@lines) {
    #print("Line: $line\n");
    if ($line =~ /Point\[\d+\]: ([-\d]+), ([-\.\d]+)$/) {
        $angle = $1;
        $cm = $2;
        #print("Data: $angle, $cm\n");
        if ($cm > 0) {
            #$x = $cm * cos( deg2rad($angle) );
            #$y = $cm * sin( deg2rad($angle) );
            $x = nearest(.001, $cm * cos( ($angle / 180.0) * pi ));
            $y = nearest(.001, $cm * sin( ($angle / 180.0) * pi ));
            #print("$x, $y\n");
            push @xpts, $x;
            push @ypts, $y;
        }
    }
}

my $img = Chart::Plot->new(800,640);
#$img->setData(\@xpts, \@ypts, 'Points Noline Blue') or die $img->error;
$img->setData(\@xpts, \@ypts, 'Points Blue') or die $img->error;

$img->setGraphOptions (
  vertGraphOffset => 20,
  horGraphOffset  => 20,
  vertAxisLabel   => "Y (meters)",
  horAxisLabel    => "X (meters)",
  title           => "LIDAR Captured Data ($filename)",
);

$out_name = $filename;
$out_name =~ s/(.)\.\w+/$1\.png/;
print("Output: $out_name\n");

open my $fh, '>:raw', $out_name or die $!;
print $fh $img->draw();
close $fh;
