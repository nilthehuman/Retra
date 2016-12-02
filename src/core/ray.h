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

// Ray class for a single ray of light
// Part of Retra, the Reference Tracer

#ifndef RETRA_RAY
#define RETRA_RAY

#include <cassert>
#include <stack>

#include "triplet.h"

namespace Retra {

    class Light;
    class LightPart;
    class Thing;
    class ThingPart;
    class Scene;

    class Ray {
    public:
        Ray( const Scene* scene, const Vector& origin, const Vector& direction, const RGB& color, int depth, double rrLimit )
            : scene( scene )
            , origin( origin )
            , direction( direction )
            , color( color )
            , depth( depth )
            , rrLimit( rrLimit )
            , lightHit( NULL )
            , lightPartHit( NULL )
            , thingHit( NULL )
            , thingPartHit( NULL )
            , insideThings()
        {
            assert( scene );
            this->direction.normalize();
        }

        const Vector& getOrigin()     const { return origin; }
        const Vector& getDirection()  const { return direction; }
        const RGB&    getColor()      const { return color; }
        int           getDepth()      const { return depth; }

        Vector operator[]( double t ) const { return origin + direction * t; }

        void    traceToNextIntersection(); // Initial bounce to find the surface point the Camera sees
        RGB     trace();                   // Trace the Ray all the way through

    private:
        void    paint( const Triplet& otherColor ) { color *= otherColor; } // Incorporate the color of a Surface that was hit

        inline bool russianRoulette() const;
        RGB     bounceDiffuse();
        RGB     bounceMetallic();
        RGB     bounceReflect();
        RGB     bounceRefract();

        static double schlick( double n1, double n2, double cosTheta );

    private:
        const Scene* const scene;

        Vector  origin;
        Vector  direction;
        RGB     color; // Current color (may change with each bounce)
        int     depth; // Number of bounces to live (at most)
        const double rrLimit; // Limit to keep weak Rays alive in Russian Roulette

        const Light*               lightHit;     // Last intersected Light
        const LightPart*           lightPartHit; // Last intersected LightPart
        const Thing*               thingHit;     // Last intersected Thing
        const ThingPart*           thingPartHit; // Last intersected ThingPart
        std::stack< const Thing* > insideThings; // LIFO stack of penetrated Things
    };

}

#endif // RETRA_RAY

