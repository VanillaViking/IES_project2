{ pkgs ? import <nixpkgs> { } }:

with pkgs;

mkShell {
  buildInputs = [ pkgsCross.avr.buildPackages.gcc pkgsCross.avr.libcCross avrdude screen ]; # your dependencies here
}
