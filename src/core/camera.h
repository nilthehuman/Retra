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

// Camera class, defines the point of view and the screen
// Part of Retra, the Reference Tracer

#ifndef RETRA_CAMERA
#define RETRA_CAMERA

#include <fstream>
#include <iostream>

#include "ray.h"
#include "triplet.h"

namespace Retra {

    class Camera {
    public:
        enum Axis { AXIS_X, AXIS_Y, AXIS_Z };

        // The GUI's going to need these
        struct RenderInfo {
            RenderInfo( int clockError, int sppSoFar, bool sceneChanged )
                : clockError( clockError )
                , sppSoFar( sppSoFar )
                , sceneChanged( sceneChanged )
            { }

            int  clockError;
            int  sppSoFar;
            bool sceneChanged;
        };

    private:
        struct Screen {
            Screen() { }
            Screen( Vector topLeft, Vector topRight, Vector bottomLeft, Vector bottomRight, int width, int height )
                : gridwidth( width )
                , gridheight( height )
            {
                window[0] = topLeft;
                window[1] = topRight;
                window[2] = bottomLeft;
                window[3] = bottomRight;
            }

            Vector window[4];
            int gridwidth;
            int gridheight;
        };

    public:
        Camera( const Scene* scene )
            : scene( scene )
            , pixels( NULL )
            , sppSoFar( 0 )
            , rendering( false )
        { }
        Camera( const Scene* scene, Vector viewpoint, Screen screen, int width, int height )
            : scene( scene )
            , viewpoint( viewpoint )
            , screen( screen )
            , sppSoFar( 0 )
            , rendering( false )
        {
            pixels = new RGB*[height];
            for ( int i = 0; i < height; ++i )
                pixels[i] = new RGB[width];
        }
        ~Camera()
        {
            if ( pixels )
            {
                for ( int i = 0; i < screen.gridheight; ++i )
                    delete[] pixels[i];
                delete[] pixels;
            }
            if ( modeFlags.verbose )
                std::cerr << "                                            " << '\r' << std::flush;
        }

        void clear();
        void capture( int spp, int depth, double rrLimit );
        const RenderInfo render( int renderTime, int depth, double rrLimit, double gamma = 1 );
        void gammaCorrect( double gamma );

        void writePixels( std::ostream& os ) const;

        void move( double delta, Axis chosenAxis );
        void turn( double theta, Axis chosenAxis );

        const Scene* getScene() const { return scene; }
        int  getGridwidth ()    const { return screen.gridwidth; }
        int  getGridheight()    const { return screen.gridheight; }
        const RGB** getPixels() const { return (const RGB**)pixels; }

        friend std::istream& operator>>( std::istream& is, Camera& camera );

    private:
        Ray generateRay( int row, int col, int depth, double rrLimit ) const;

    private:
        const Scene* const scene;

        Vector viewpoint;
        Screen screen;
        RGB**  pixels; // The end results go here
        int    sppSoFar;
        bool   rendering;
    };

}

#endif // RETRA_CAMERA

