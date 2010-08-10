#!/usr/bin/env perl
# vim:ts=4:sw=4:expandtab
# © 2010 Michael Stapelberg
# spielt den teil des mikrokontrollers innen nach

use strict;
use warnings;
use Time::HiRes qw(sleep time);
use autodie;
use Device::SerialPort;
use v5.10;
use EV;
use POSIX;
use IO::Handle;

setlocale(LC_NUMERIC, "C");

$| = 1;

sub buffer {
    my ($port, $dir) = @_;
    my ($blocking, $in, $out, $error) = $port->status;
    ($dir eq 'in' ? $in : $out)
}

my $port = "/dev/ttyUSB0";
say "Opening serial port $port";
my $p = Device::SerialPort->new($port) or die "$!\n";
$p->baudrate(9600);

my $ping_time;
my $ping_bits;
my $ping_timer = EV::timer 1, 2, sub {
    my $bits = chr(int(rand(26)) + 65) . chr(int(rand(26)) + 65);
    $p->write('^PING ' . $bits . '$');
    $p->write_drain while buffer($p, 'out') > 0;

    $ping_time = time();
    $ping_bits = $bits;
};

my $input_buffer = '';

# correctly store new bytes in $input_buffer, synchronizing on the start of
# packet byte, then call handle_message() with the payload
sub new_byte {
    my ($byte) = @_;
    return if length($input_buffer) == 0 and $byte ne '^';
    $input_buffer .= $byte;
    return unless length($input_buffer) == 10;
    if (substr($input_buffer, 8, 1) ne '$') {
        warn 'Invalid packet: ' . $input_buffer;
        return;
    }
    {
        local $/;
        $/ = "\r\n";
        chomp $input_buffer;
    }
    handle_message(substr($input_buffer, 1, length($input_buffer)-2));
    $input_buffer = '';
}

my $pin_buffer = '';
sub handle_message {
    my ($msg) = @_;
    if ($msg =~ /^PAD /) {
        my $key = substr($msg, 4, 1);
        say "key $key was pressed";
        if ($key eq '#') {
            handle_pin($pin_buffer);
            $pin_buffer = '';
        } else {
            $pin_buffer .= $key;

            # send the beep and light the LED
            $p->write("^LED 1 1\$\n");
            $p->write("^BEEP 1 \$\n");
            $p->write_drain while buffer($p, 'out') > 0;
        }
        return;
    }

    if ($msg =~ /^PONG/) {
        my ($bits) = ($msg =~ /^PONG ([A-Z]{2})\$/);
        if ($bits ne $ping_bits) {
            warn 'Bits are not equal (' . $bits . ' (recv) vs. ' . $ping_bits . ' (sent))';
        }
        say "received pong ($msg) after " . (time() - $ping_time) . " sec";
        return;
    }

    say "payload: $msg";
}

sub handle_pin {
    my ($pin) = @_;

    say "PIN $pin was entered";
    if ($pin eq '3344') {
        $p->write("^LED 1 2\$\n");
        $p->write_drain while buffer($p, 'out') > 0;
    } else {
        $p->write("^LED 2 2\$\n");
        $p->write("^BEEP 2 \$\n");
        $p->write_drain while buffer($p, 'out') > 0;
    }
}

# When idle, check the serial line buffer for new bytes and handle them
my $check = EV::idle sub {
    return unless buffer($p, 'in') > 0;
    my ($cnt, $str) = $p->read(1);
    exit 1 if $cnt != 1;

    new_byte($str);
};

EV::loop
