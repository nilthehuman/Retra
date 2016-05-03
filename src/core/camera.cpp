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

// Camera class methods
// Part of Retra, the Reference Tracer

#include "camera.h"

#include <ctime>
#include <limits>

#include <omp.h>

#include "scene.h"

namespace Retra {

    // Reset pixel buffer to all black
    void Camera::clear()
    {
        assert( !rendering );
        #pragma omp parallel for
        for ( int row = 0; row < screen.gridheight; ++row )
            for ( int col = 0; col < screen.gridwidth; ++col )
                pixels[row][col] = RGB::Black;
        sppSoFar = 0;
        if ( modeFlags.verbose )
            std::cerr << "                                            " << '\r' << std::flush;
    }

    // Take a shot of the virtual Scene
    void Camera::capture( int spp, int depth, double rrLimit )
    {
        rendering = true;
        const time_t start = std::time( NULL );
        #pragma omp parallel
        {
            const int numThreads = omp_get_num_threads();
            const int threadNum  = omp_get_thread_num();
            const int rowBegin   = (double) threadNum      / numThreads * screen.gridheight;
            const int rowEnd     = (double)(threadNum + 1) / numThreads * screen.gridheight;
            // The engine's core loop
            for ( int row = rowBegin; row < rowEnd; ++row )
            {
                if ( modeFlags.verbose && 2 == threadNum ) // Usually the lower mid thread has the best idea of progress
                {
                    std::cerr << "Camera: rendering... " << (int)((double)(row - rowBegin) / (rowEnd - rowBegin) * 100) << "% done.";
                    if ( 3 == row % 4 )
                    {
                        const int estimate = (int)( difftime( std::time( NULL ), start ) * (rowEnd - row) / (row - rowBegin) );
                        const int m = estimate / 60;
                        const int s = estimate % 60;
                        std::cerr << " Time left: ";
                        if ( m ) std::cerr << m << " min(s) and ";
                        std::cerr << s << " sec(s).              ";
                    }
                    std::cerr << '\r' << std::flush;
                }
                for ( int col = 0; col < screen.gridwidth; ++col )
                {
                    Triplet pixelColorSum; // Adding up the results would yield an invalid (out-of-bounds) RGB
                    Ray ray = generateRay( row, col, depth, rrLimit );
                    ray.traceToNextIntersection(); // Initialize ray (sort of)
                    if ( ray.getDepth() )
                    {
                        for ( int s = 0; s < spp; ++s )
                        {
                            Ray rayClone( ray );
                            pixelColorSum += rayClone.trace();
                        }
                        pixels[row][col] = pixelColorSum / spp;
                    }
                    else
                        pixels[row][col] = ray.getColor();
                }
            }
        }
        if ( modeFlags.verbose )
            std::cerr << "\rCamera: rendering completed.   " << std::endl;
        rendering = false;
    }

    // Keep refining the image until a given time limit is reached
    const Camera::RenderInfo Camera::render( int renderTime, int depth, double rrLimit, double gamma )
    {
        const bool sceneChanged = scene->isChanged();
        if ( sceneChanged )
        {
            clear();
            scene->clearChanged();
        }
        rendering = true;
        const int sppBefore = sppSoFar;
        int elapsedTime = 0;
        int clockError  = 0;
        Triplet* pixelColorSum = new Triplet[ screen.gridwidth * screen.gridheight ];
        const clock_t start = clock();
        #pragma omp parallel shared(elapsedTime, clockError, pixelColorSum)
        {
            const int numThreads = omp_get_num_threads();
            const int threadNum  = omp_get_thread_num();
            const int rowBegin   = (double) threadNum      / numThreads * screen.gridheight;
            const int rowEnd     = (double)(threadNum + 1) / numThreads * screen.gridheight;
            for ( int row = rowBegin; row < rowEnd; ++row )
                for ( int col = 0; col < screen.gridwidth; ++col )
                    pixelColorSum[row*screen.gridwidth + col] = RGB::Black;
            while ( elapsedTime < renderTime )
            {
                for ( int row = rowBegin; row < rowEnd; ++row )
                {
                    if ( modeFlags.verbose && 0 == threadNum )
                        std::cerr << "Camera: rendering... " << sppSoFar << " samples per pixel." << '\r' << std::flush;
                    for ( int col = 0; col < screen.gridwidth; ++col )
                    {
                        Ray ray = generateRay( row, col, depth, rrLimit );
                        ray.traceToNextIntersection(); // Initialize ray (sort of)
                        pixelColorSum[row*screen.gridwidth + col] += ray.trace();
                    }
                }
                if ( 0 == threadNum )
                {
                    ++sppSoFar;
                    elapsedTime = 1000.0 * (clock() - start) / CLOCKS_PER_SEC / numThreads; // clock() returns total CPU time
                }
                #pragma omp flush(elapsedTime)
                #pragma omp barrier
            }
            for ( int row = rowBegin; row < rowEnd; ++row )
                for ( int col = 0; col < screen.gridwidth; ++col )
                    // Take the SPP-weighed average of the old and new color values
                    pixels[row][col] = pixels[row][col] * ((double)sppBefore / sppSoFar) +
                                       RGB(pixelColorSum[row*screen.gridwidth + col] / (sppSoFar - sppBefore)).gamma(gamma) * ((double)(sppSoFar - sppBefore) / sppSoFar);
            if ( 0 == threadNum )
            {
                elapsedTime = 1000.0 * (clock() - start) / CLOCKS_PER_SEC / numThreads; // clock() returns total CPU time
                clockError = elapsedTime * (numThreads - 1);
            }
        }
        delete[] pixelColorSum;
        const RenderInfo renderInfo( clockError, sppSoFar, sceneChanged );
        rendering = false;
        return renderInfo;
    }

    void Camera::gammaCorrect( double gamma )
    {
        assert( !rendering );
        if ( 1 == gamma )
            return;
        #pragma omp parallel for
        for ( int row = 0; row < screen.gridheight; ++row )
            for ( int col = 0; col < screen.gridwidth; ++col )
                pixels[row][col].gamma( gamma );
    }

    // Shoot ray from camera viewpoint through a given pixel
    Ray Camera::generateRay( int row, int col, int depth, double rrLimit ) const
    {
        const Vector leftEdge  = screen.window[0] + (screen.window[2] - screen.window[0]) * ((0.5 + row) / screen.gridheight );
        const Vector origin    = leftEdge + (screen.window[1] - screen.window[0]) * ((0.5 + col) / screen.gridwidth );
        const Vector direction = origin - viewpoint;
        // Camera emits white Rays as if it's the real light source and Lights are the sinks.
        // This will produce the same RGB results as long as Ray::paint is commutative
        return Ray( scene, origin, direction, RGB::White, depth, rrLimit );
    }

    // Write rendering results to character stream in PPM format
    void Camera::writePixels( std::ostream& os ) const
    {
        assert( !rendering );
        if ( modeFlags.verbose )
            std::cerr << "Camera: writing pixels to stream... ";
        os << "P3\n" << screen.gridwidth << " " << screen.gridheight << "\n" << 255 << "\n";
        for ( int row = 0; row < screen.gridheight; ++row )
            for ( int col = 0; col < screen.gridwidth; ++col )
            {
                Triplet rgb255 = pixels[row][col].project( 0, 255 );
                os << int(rgb255.x + 0.5) << " " << int(rgb255.y + 0.5) << " " << int(rgb255.z + 0.5) << " ";
            }
        if ( modeFlags.verbose )
            std::cerr << "done." << std::endl;
    }

    // Translate both the viewpoint and the screen in camera space
    void Camera::move( double delta, Axis chosenAxis )
    {
        assert( !rendering );
        clear();
        // Screen position relative to the viewpoint
        const Screen relScreen( screen.window[0] - viewpoint,
                                screen.window[1] - viewpoint,
                                screen.window[2] - viewpoint,
                                screen.window[3] - viewpoint,
                                0, 0 );
        Vector translation;
        switch ( chosenAxis )
        {
            case AXIS_X:
                translation = (relScreen.window[1] - relScreen.window[0]).normalized();
                break;
            case AXIS_Y:
                translation = (relScreen.window[0] - relScreen.window[2]).normalized();
                break;
            case AXIS_Z:
                translation = (relScreen.window[0] + relScreen.window[1] +
                               relScreen.window[2] + relScreen.window[3]).normalized() * -1.0;
                break;
            default:
                assert( false );
        }
        viewpoint += translation * delta;
        for ( int i = 0; i < 4; ++i )
            screen.window[i] += translation * delta;
    }

    // Rotate the screen in the positive direction around one of the camera axes
    void Camera::turn( double theta, Axis chosenAxis )
    {
        assert( !rendering );
        clear();
        // Screen position relative to the viewpoint
        const Screen relScreen( screen.window[0] - viewpoint,
                                screen.window[1] - viewpoint,
                                screen.window[2] - viewpoint,
                                screen.window[3] - viewpoint,
                                0, 0 );
        // The actual axis direction
        Vector axisUnit;

        // The Camera always faces the -Z direction
        switch ( chosenAxis )
        {
            case AXIS_X:
                axisUnit = (relScreen.window[1] - relScreen.window[0]).normalized();
                break;
            case AXIS_Y:
                axisUnit = (relScreen.window[0] - relScreen.window[2]).normalized();
                break;
            case AXIS_Z:
                axisUnit = (relScreen.window[0] + relScreen.window[1] +
                            relScreen.window[2] + relScreen.window[3]).normalized() * -1.0;
                break;
            default:
                assert( false );
        }

        // https://www.fastgraph.com/makegames/3drotation/
        const double c = cos( theta );
        const double s = sin( theta );
        const double t = 1 - c;
        // The rotation matrix
        Vector rotation[3] = {
            Vector( t*axisUnit.x*axisUnit.x + c, t*axisUnit.x*axisUnit.y - s*axisUnit.z, t*axisUnit.x*axisUnit.z + s*axisUnit.y ),
            Vector( t*axisUnit.x*axisUnit.y + s*axisUnit.z, t*axisUnit.y*axisUnit.y + c, t*axisUnit.y*axisUnit.z - s*axisUnit.x ),
            Vector( t*axisUnit.x*axisUnit.z - s*axisUnit.y, t*axisUnit.y*axisUnit.z + s*axisUnit.x, t*axisUnit.z*axisUnit.z + c )
        };

        Vector topLeft(
            rotation[0].x * relScreen.window[0].x + rotation[0].y * relScreen.window[0].y + rotation[0].z * relScreen.window[0].z,
            rotation[1].x * relScreen.window[0].x + rotation[1].y * relScreen.window[0].y + rotation[1].z * relScreen.window[0].z,
            rotation[2].x * relScreen.window[0].x + rotation[2].y * relScreen.window[0].y + rotation[2].z * relScreen.window[0].z
        );
        Vector topRight(
            rotation[0].x * relScreen.window[1].x + rotation[0].y * relScreen.window[1].y + rotation[0].z * relScreen.window[1].z,
            rotation[1].x * relScreen.window[1].x + rotation[1].y * relScreen.window[1].y + rotation[1].z * relScreen.window[1].z,
            rotation[2].x * relScreen.window[1].x + rotation[2].y * relScreen.window[1].y + rotation[2].z * relScreen.window[1].z
        );
        Vector bottomLeft(
            rotation[0].x * relScreen.window[2].x + rotation[0].y * relScreen.window[2].y + rotation[0].z * relScreen.window[2].z,
            rotation[1].x * relScreen.window[2].x + rotation[1].y * relScreen.window[2].y + rotation[1].z * relScreen.window[2].z,
            rotation[2].x * relScreen.window[2].x + rotation[2].y * relScreen.window[2].y + rotation[2].z * relScreen.window[2].z
        );
        Vector bottomRight(
            rotation[0].x * relScreen.window[3].x + rotation[0].y * relScreen.window[3].y + rotation[0].z * relScreen.window[3].z,
            rotation[1].x * relScreen.window[3].x + rotation[1].y * relScreen.window[3].y + rotation[1].z * relScreen.window[3].z,
            rotation[2].x * relScreen.window[3].x + rotation[2].y * relScreen.window[3].y + rotation[2].z * relScreen.window[3].z
        );
        screen.window[0] = viewpoint + topLeft;
        screen.window[1] = viewpoint + topRight;
        screen.window[2] = viewpoint + bottomLeft;
        screen.window[3] = viewpoint + bottomRight;
    }

}

