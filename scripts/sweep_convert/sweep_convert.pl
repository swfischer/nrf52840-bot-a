#!/usr/bin/perl

use Chart::Gnuplot;
use Math::Trig;
use Math::Round qw(:all); # For nearest()

sub help {
    print("usage: sweep_convert3.pl <filename>\n");
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

$out_name = $filename;
$out_name =~ s/(.)\.\w+/$1\.png/;
print("Output File: $out_name\n");

my $chart = Chart::Gnuplot->new(
    output => "$out_name",
    title  => "LIDAR Captured Data ($filename)",
    xlabel => "meters",
    ylabel => "meters",
    xrange => [-7,7],
    xrange => [-7,7],
    bg => "white",
    grid => {
        linetype => 'dash',
    }
);

my $dataSet = Chart::Gnuplot::DataSet->new(
    xdata => \@xpts,
    ydata => \@ypts,
    color => "blue",
    pointtype => 0,
    style => "linespoints",
);

$chart->plot2d($dataSet);
