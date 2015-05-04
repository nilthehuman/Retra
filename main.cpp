/*
 * Copyright 2015 Dániel Arató
 *
 * This file is part of Retra.
 *
 * Retra is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Retra is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Retra.  If not, see <http://www.gnu.org/licenses/>.
 */

// The main function. Parses command line flags, reads the input scene file and runs the actual tracer
// Part of Retra, the Reference Tracer

#include <iostream>
#include <limits>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <cstdlib>
#include <ctime>

#include "core/camera.h"
#include "core/scene.h"

#ifdef COMPILE_WITH_GUI
#include "gui/gui.h"
#endif

#include "parsescene/parsescene.h"

using namespace Retra;

const std::string VERSION = "1.2.0";

struct flags Retra::modeFlags;

struct arguments {
    char*  progname;
    int    spp;
    int    depth;
    double rrLimit;
    double gamma;
    char*  inFilename;
    char*  outFilename;
#ifdef COMPILE_WITH_GUI
    bool   gui;
    int    fps;
#endif
};

void help( std::string progname )
{
    std::cout << "usage: " << progname << " SCENE_FILENAME [OPTIONS]" << std::endl << std::endl;
    std::cout << "Command line options:" << std::endl;
    std::cout << "  -s, --spp SAMPLES   Set the number of samples per pixel (default 64)" << std::endl;
    std::cout << "  -d, --depth DEPTH   Set the maximal depth (length) of any path (default 12)" << std::endl;
    std::cout << "  -r, --rr LIMIT      Set the stay-alive limit used in Russian roulette (default 0.25)" << std::endl;
    std::cout << "  -g, --gamma EXP     Set the exponent for post-mortem gamma correction (default 1.0)" << std::endl;
    std::cout << "  -o, --out FILENAME  Set the filename for the output image (default image.ppm)" << std::endl;
#ifdef COMPILE_WITH_GUI
    std::cout << "      --gui           Start interactive graphical interface instead of outputting to file" << std::endl;
    std::cout << "  -f, --fps FPS       Set the framerate for the graphical interface (default 10)" << std::endl;
#endif
    std::cout << "  -h, --help          Print this help message and quit" << std::endl;
    std::cout << "  -v, --version       Print version information and quit" << std::endl << std::endl;
    std::cout << "Exit status:" << std::endl;
    std::cout << "  0  if OK" << std::endl;
    std::cout << "  1  if arguments are unparseable" << std::endl;
    std::cout << "  2  if input file is unreadable" << std::endl;
    std::cout << "  3  if input file is unparseable" << std::endl;
    std::cout << "  4  if output file is unwriteable" << std::endl;
    exit(0);
}

void version()
{
    std::cout << "Retra " << VERSION << std::endl;
    std::cout << "License GPLv3+: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>." << std::endl;
    std::cout << "This is free software: you are free to change and redistribute it." << std::endl;
    std::cout << "There is NO WARRANTY, to the extent permitted by law." << std::endl;
    exit(0);
}

void usage( std::string progname )
{
    std::cerr << "usage: " << progname << " SCENE_FILENAME [-v|--verbose] [--spp SAMPLES_PER_PIXEL]" << std::endl;
    std::cerr << "  [--depth MAX_DEPTH_OF_PATHS] [--rr RUSSIAN_ROULETTE_LIMIT] [--gamma GAMMA]" << std::endl;
    std::cerr << "  [--out IMAGE_FILENAME] [--gui]" << std::endl;
    exit(1);
}

void die( int exitCode, std::string message )
{
    std::cerr << "error: " << message << std::endl;
    exit( exitCode );
}

// Read and process the command line arguments to the program
void parseArgs( int argc, char* argv[], struct arguments* args )
{
    args->progname = argv[0];
#ifdef COMPILE_WITH_GUI
    if ( std::string(args->progname).find("gui") != std::string::npos )
        args->gui = true;
#endif
    for ( int i = 1; i < argc; ++i )
    {
        if( !strcmp(argv[i], "-s") || !strcmp(argv[i], "--spp") )
        {
            if ( argc <= ++i )
                usage( args->progname );
            args->spp = atoi( argv[i] );
            if ( !args->spp )
                usage( args->progname );
        }
        else if( !strcmp(argv[i], "-d") || !strcmp(argv[i], "--depth") )
        {
            if ( argc <= ++i )
                usage( args->progname );
            args->depth = atoi( argv[i] );
            if ( !args->depth )
                usage( args->progname );
        }
        else if( !strcmp(argv[i], "-r") || !strcmp(argv[i], "--rr") )
        {
            if ( argc <= ++i )
                usage( args->progname );
            args->rrLimit = atof( argv[i] );
            if ( !args->rrLimit )
                usage( args->progname );
        }
        else if( !strcmp(argv[i], "-g") || !strcmp(argv[i], "--gamma") )
        {
            if ( argc <= ++i )
                usage( args->progname );
            args->gamma = atof( argv[i] );
            if ( !args->gamma )
                usage( args->progname );
        }
        else if( !strcmp(argv[i], "-o") || !strcmp(argv[i], "--out") )
        {
            if ( argc <= ++i )
                usage( args->progname );
            args->outFilename = argv[i];
        }
#ifdef COMPILE_WITH_GUI
        else if( !strcmp(argv[i], "--gui") )
        {
            args->gui = true;
        }
        else if( !strcmp(argv[i], "-f") || !strcmp(argv[i], "--fps") )
        {
            if ( argc <= ++i )
                usage( args->progname );
            args->fps = atoi( argv[i] );
            if ( !args->fps )
                usage( args->progname );
        }
#endif
        else if( !strcmp(argv[i], "-h") || !strcmp(argv[i], "--help") )
        {
            help( args->progname );
        }
        else if( !strcmp(argv[i], "-v") || !strcmp(argv[i], "--verbose") )
        {
            modeFlags.verbose = true;
        }
        else if( !strcmp(argv[i], "--version") )
        {
            version();
        }
        else
        {
            if( args->inFilename )
                usage( args->progname );
            args->inFilename = argv[i];
        }
    }
    if( !args->inFilename )
        usage( args->progname );
    if( 1.0 < args->rrLimit )
    {
        std::cerr << "main: warning: an rrLimit higher than 1.0 will produce unrealistic results." << std::endl;
        std::cerr << "               This may not be what you intend." << std::endl;
    }
#ifdef COMPILE_WITH_GUI
    if( args->gui )
    {
        if( args->spp   != 64 )
            std::cerr << "main: warning: starting in GUI mode, disregarding --spp setting." << std::endl;
        if( args->gamma !=  1 )
            std::cerr << "main: warning: starting in GUI mode, disregarding --gamma setting." << std::endl;
        if( strcmp( args->outFilename, "image.ppm" ) )
            std::cerr << "main: warning: starting in GUI mode, disregarding --out setting." << std::endl;
    }
#endif
}

Camera* camera = NULL;
void cleanup()
{
    delete camera->getScene();
    delete camera;
}

int main( int argc, char* argv[] )
{
    // Set mode flags to default values
    modeFlags.verbose = false;

    // Parse command line arguments
    struct arguments args;
    args.spp         = 64;
    args.depth       = 12;
    args.rrLimit     = 0.25;
    args.gamma       = 1;
    args.inFilename  = NULL;
    args.outFilename = (char*)"image.ppm";
#ifdef COMPILE_WITH_GUI
    args.gui         = false;
    args.fps         = 10;
#endif
    parseArgs( argc, argv, &args );
    if( modeFlags.verbose )
    {
        std::cerr << "main: arguments: ";
        std::cerr << "spp = " << args.spp << ", depth = " << args.depth << ", rrLimit = " << args.rrLimit << ", gamma = " << args.gamma << "," << std::endl;
#ifdef COMPILE_WITH_GUI
        if( !args.gui )
#endif
            std::cerr << "      outFilename = " << args.outFilename << std::endl;
    }

    // Process scene description input
    if( modeFlags.verbose )
    {
        if( !strcmp(args.inFilename, "-") )
            std::cerr << "main: reading scene description from standard input..." << std::endl;
        else
            std::cerr << "main: reading scene file '" << args.inFilename << "'..." << std::endl;
    }
    std::istream* is;
    std::ifstream ifs;
    if( !strcmp(args.inFilename, "-") )
    {
        is = &std::cin;
    }
    else
    {
        ifs.open( args.inFilename );
        if( !ifs.is_open() )
            die( 2, "cannot read file at '" + std::string(args.inFilename) + "'" );
        is = &ifs;
    }
    try {
        camera = parseScene( *is );
    }
    catch( std::string e ) {
        die( 3, e );
    }
    if( ifs.is_open() )
        ifs.close();
    if( modeFlags.verbose )
        std::cerr << "main: input scene file read successfully." << std::endl;

#ifdef COMPILE_WITH_GUI
    if ( args.gui )
    {
        if ( modeFlags.verbose )
            std::cerr << "main: creating GUI." << std::endl;
        GUI gui( camera );
        gui.initialize( &argc, argv );
        gui.setup( args.depth, args.rrLimit, (int)(1000.0 / args.fps) );
        atexit( &cleanup );
        if ( modeFlags.verbose )
            std::cerr << "main: starting the renderer." << std::endl;
        gui.run();
    }
#endif

    if( strcmp(args.outFilename, "-") )
    {
        // Check if target file is writable before starting calculations
        if( modeFlags.verbose )
            std::cerr << "main: checking if output file is writable... ";
        std::ofstream ofs;
        ofs.open( args.outFilename );
        if( !ofs.is_open() )
            die( 4, "cannot write file at '" + std::string(args.outFilename) + "'" );
        ofs.close();
        if( modeFlags.verbose )
            std::cerr << "OK." << std::endl;
    }

    // Render image
    if ( modeFlags.verbose )
        std::cerr << "main: starting the renderer." << std::endl;
    const time_t start = std::time( NULL );
    std::srand( start );
    camera->capture( args.spp, args.depth, args.rrLimit ); // Trace those suckers
    camera->gammaCorrect( args.gamma );
    if ( modeFlags.verbose )
    {
        const time_t end = std::time( NULL );
        const int m =      difftime(end, start) / 60;
        const int s = (int)difftime(end, start) % 60;
        std::cerr << "main: rendering took ";
        if ( m ) std::cerr << m << " minute(s) and " ;
        std::cerr << s << " second(s)." << std::endl;
    }

    if( strcmp(args.outFilename, "-") )
    {
        // Dump results into file
        std::ofstream ofs;
        ofs.open( args.outFilename );
        while( !ofs.is_open() )
        {
            std::cerr << "cannot write file at '" << args.outFilename << "'; please make the file writable and press Return." << std::endl;
            std::cin.ignore();
            ofs.open( args.outFilename );
        }
        camera->writePixels( ofs );
        ofs.close();
        if ( modeFlags.verbose )
            std::cerr << "main: image written to '" << args.outFilename << "'" << std::endl;
    }
    else
    {
        // Dump results to standard output
        camera->writePixels( std::cout );
        if ( modeFlags.verbose )
            std::cerr << "main: image written to standard output" << std::endl;
    }

    cleanup();
    return 0;
}

