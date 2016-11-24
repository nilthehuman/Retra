# __Retra__, the Reference Tracer

Version 1.3.0

(__Warning__: this README was last updated in November 2016 and may be out of
date.)

### What it is

__Retra__ is a minimal, unoptimized CPU path tracer written in portable C++98.
It produces approximate images of a 3D scene of spheres, planes and triangles.

__Retra__ is the sister project of another small rendering program called
__Silence__. __Retra__'s main purpose is to serve as a reference and benchmark
for __Silence__. They share the same featureset but __Retra__ is a vanilla path
tracer while __Silence__ uses a different, experimental algorithm.

### What it is not

__Retra__ is not a practical or high performance rendering tool. It is a
straightforward implementation of the traditional eye ray tracing algorithm
and a baseline for the development and testing of __Silence__.

__Retra__ is not a valid solution for your raytracing assignment. As always you
are welcome to read the source, make changes, experiment and ask questions but
you probably got that assignment in the first place because you want to learn
about computer graphics or prove that you can build a path tracer on your own.

### I need __Silence__, not this bullshit

Head on over to <https://github.com/nilthehuman/Silence> where __Silence__ and
its full source code is available under the same license terms as __Retra__.

## Installation

### Building __Retra__ from source

The best way to install __Retra__ is to build it from source on your machine.
The following instructions are for Unix-like operating systems but __Retra__
should be easy to build on Windows machines too with [MinGW][1]. You may have
to change the line endings from LF to CRLF on Windows.
The source is available on GitHub or as a tarball from a web server run by the
Budapest University of Technology and Economics. Choose whatever is convenient
for you.

#### A. / Cloning from GitHub

Go to the directory you want to download to and copy the git repo like so:
```bash
git clone git://github.com/nilthehuman/Retra.git
```

#### B. / Downloading the tarball

Go to the directory you want to download to, get the tarball and extract it:
```bash
wget users.hszk.bme.hu/~ad767/Retra_src.tar.gz
tar -xvf Retra_src.tar.gz
```

#### Compiling

A Makefile is provided with the source so you don't have to build the program
by hand. (GNU `make` is of course available in the official package repositories
of all common GNU/Linux distributions or in the [MinGW suite][1] for Windows.)

To compile __Retra__ with no graphical interface issue the following command in
the folder you downloaded it to:
```bash
make cli
```

Or, to compile _with_ graphical interface:
```bash
make gui
```

(Simply typing `make` builds both versions.)

If you see `==== Retra built successfully ====` you're golden. If the compiler
(or linker) complains about undefined stuff please refer to the "Dependencies"
section below.

The Makefile specifies g++ as the compiler. If you want to use a different
compiler just change the first two lines of the Makefile accordingly.
The following compilers have been tested and should work fine:
  * clang++ ([with OpenMP][2])
  * g++

### Pre-built binaries

You can find a tarball of pre-built binary executables on BUTE's web server:

```bash
wget users.hszk.bme.hu/~ad767/Retra_bin.tar.gz
tar -xvf Retra_bin.tar.gz
```

These binaries were compiled and linked on my Debian Jessie machine. If you're
having issues using them on your own system please try compiling __Retra__ from
source instead (see above).

## Dependencies

__Retra__ relies on...

  * [the C standard libraries][3] for math, string functions, timing and assert
  * [the C++ Standard Library][4] for stream I/O, containers and iterators
  * [OpenMP][5] for parallel processing

That's it, that's all you need to render to file. Since GCC comes with all of
these built-in you've probably already got them.

The graphical interface has the following additional dependencies:

  * OpenGL or an equivalent replacement (like [Mesa][6])
  * The OpenGL Utility Toolkit or an equivalent replacement (like [freeglut][7])

Both should be available in your favourite distro's official repositories.

## Usage

### Basic

Invoking __Retra__ as follows will output an image to `image.ppm`:

```bash
./retra scene.json
```

It is recommended to use the verbose flag to help with troubleshooting and show
you an estimate of the remaining time:

```bash
./retra scene.json -v
```

### Options

__Retra__ is a bare-bones path tracer and doesn't have too many knobs on it.
Basically you can set the number of __samples per pixel__, the upper limit on
__recursion depth__, a factor to fine-tune __Russian roulette__ and the __gamma
correction__ exponent.
Type `./retra -h` for a full list of command line options.

Example to render with 300 samples per pixel, paths no deeper than 5, a rather
forgiving Russian roulette and no gamma correction:

```bash
./retra scene.json -v -s 300 -d 5 -r 0.125 -g 1
```

Adding depth is much cheaper than adding SPP so if you're unsatisfied with the
rendering quality maybe try that first.

## Features

### The common featureset of __Retra__ and __Silence__

Below is a list of "external" features (stuff the user cares about) provided by
both __Retra__ and __Silence__.

  * Global illumination (obviously)
  * Three kinds of unscaled surface primitives:
    - Spheres
    - Planes
    - Triangles
  * Four kinds of basic BRDF's that can be combined to form more complicated
materials:
    - Diffuse
    - Metallic (Fresnel)
    - Reflecting
    - Refractive (dielectric)
  * Point lights
  * Area lights
  * Soft shadows
  * Gamma correction
  * A simple scene description format based on JSON
  * Image output in trivial image file format PPM
  * Graphical interface to display results on-the-fly
  * Some basic predefined object motions to demonstrate dynamic scenes
  * Verbose mode for troubleshooting and displaying an estimate of the remaining
rendering time

### "Internal" features of __Retra__

These are mainly implementation details users need not concern themselves with.

  * Explicit light sampling
  * Background objects vs foreground objects
  * Back-face culling
  * A Russian roulette variant for cutting paths short
  * Multi-threading (OpenMP)

### Optional later additions

These additional features might be added to the common featureset above (and
therefore to _both_ programs) at some later point, or they might not. Right now
they are all unsupported.

  * Ambient light
  * Octree / BSP tree

### Unfeatures

__Retra__ purposely does not and will not include the following advanced
features.
This is for the sake of simplicity and keeping both the codebase and the set of
features small. (See the introductory section "What it is".)

  * Bidirectional path tracing
  * Metropolis light transport
  * Photon mapping
  * Spectral rendering
  * Blurred reflections
  * Depth of field effect
  * Object transformations
  * Anti-aliasing
  * Texture mapping
  * Bump mapping

## Known bugs and shortcomings

__Retra__ aims to be correct and unbiased, but it is not perfect and has some
bugs. Please do NOT rely on __Retra__ to produce ground truth renders.

  * Point lights generate no caustics. (The engine does not shoot light rays)
  * Caustics in general take unreasonable amounts of samples to render (Again,
this is a property of the algorithm)
  * __Retra__ assumes that all rays are spawned in vacuum so if your camera is
inside a glass object it will look like the object is vacuum and it's _the
world_ that's made of glass

## Source tree

The __Retra__ source code and some marginal assets are organized as follows.

  * `src/core/`    -- Here be classes. All the actual image synthesis happens
  here. If you wish to use __Retra__'s engine in your own program, this is
  the part you need.
  * `src/gui/`     -- An interactive window where the image is rendered
  on-the-fly.
  * `src/parser/`  -- A non-strict good-enough parser for __Retra__ scene files.
  * `src/main.cpp` -- Provides a simple command line interface to __Retra__ or
  activates the GUI if required.
  See `./retra -h` for options.
  * `scenes/`      -- Some sample scene description files to get you started.
  A Cornell box and a few similar closed scenes with spheres.

Useful rules of thumb for keeping the interdependencies straight:

  * Source files in `src/core/` may not depend on files outside of `src/core/`.
  * Source files in `src/gui/` may not depend on files outside of `src/core/`
  and `src/gui/`.
  * Source files in `src/parser/` may not depend on files outside of `src/core/`
  and `src/parser/`. (An exception is `src/parser/parsemotion.cpp` depending on
  `src/gui/motion.h`.)

## Forking and Contributing

Fork away to your heart's content. [The License][8] not only allows but
encourages it. Do something amazing!

Pull requests are welcome too as long as they don't inflate the codebase, but I
may not always be quick to pull and merge your changes.
If you want to add features listed under "Unfeatures" please fork the project
instead.

## License

__Retra__ is free software released under the [GNU General Public License
version 3][8].
Please refer to the file named COPYING for more information.

Copyright 2015 Dániel Arató

## Frequently Asked Questions

#### __Q.__: How do I contact the maintainer?
__A.:__ You can reach me either in email or on GitHub.

Email: nil.the.human at gmail dot com

GitHub: https://github.com/nilthehuman

#### __Q.__: __Retra__ won't parse my scene file right.
__A.:__ You're probably missing a comma (`,`) between entries or a closing brace
(`}`) somewhere. Also please note that JSON uses brackets (`[`...`]`) instead of
parentheses (`(`...`)`) for tuples and lists. Try opening the file in a
syntax-aware editor.
Then again, the parser isn't perfect so if you've checked everything and you
still get an error please drop me a mail or a message.

#### __Q.__: The --fps option doesn't seem to work.
__A.:__ Your fps setting must be too high. __Retra__ will do its best to make
the rendering loop fit in the specified time, but if 1/fps seconds is too short
to sample every pixel _once_ it will go through with the loop anyway.

#### __Q.__: Performance _sucks_.
__A.:__ __Retra__ is not an industry grade renderer and features no
optimizations to the basic path tracing algorithm. The engine source is some
1300 actual lines of code.
This is in keeping with the project's aim to be a compact benchmark for
__Silence__. If you're trying to use a small pure-CPU toy to render blockbuster
animations that's your problem anyway.

#### __Q.__: Did you seriously write yet another basic path tracer instead of taking an already existing one?
__A.:__ Yes. There are three reasons for this:
  1. Having an open source path tracer with _the exact same_ list of features
as __Silence__
  2. Basing much of the __Silence__ source code off of __Retra__
  3. Using the lessons learnt from __Retra__ in __Silence__

The first two of these are key because they allow us readily to compare the
algorithm used in __Silence__ to the most widely used traditional Monte Carlo
method.

#### __Q.__: Why did you even bother specifying a license?
__A.:__ Good question! I understand you're tired of legal overhead and all the
free/non-free drama, but if you make programs or other digital content you
_should_ license them. In most jurisdictions any written work (programs
included) is [automatically copyrighted][9] once it is produced unless the
author expressly tells you how you can use it. Basically if you fail to license
your program and someone else downloads it and starts playing with the source
code they are already _commiting a crime_!
_Any_ free license is better than not having a license at all. If you can't be
bothered to care either way, just [slap a license on your work][10]. Pick GPL.
Or pick one at random. You'll be doing everyone a solid.
([Further reading.][11])

[1]:  http://www.mingw.org/wiki/HOWTO_Install_the_MinGW_GCC_Compiler_Suite
[2]:  http://clang-omp.github.io/
[3]:  http://www.gnu.org/software/libc/
[4]:  https://gcc.gnu.org/libstdc++/
[5]:  https://gcc.gnu.org/onlinedocs/libgomp/Enabling-OpenMP.html
[6]:  http://www.mesa3d.org/
[7]:  http://sourceforge.net/projects/freeglut/
[8]:  https://www.gnu.org/copyleft/gpl.html
[9]:  https://help.github.com/articles/open-source-licensing/#what-happens-if-i-dont-choose-a-license
[10]: http://choosealicense.com/
[11]: http://www.softwareliberty.com/
