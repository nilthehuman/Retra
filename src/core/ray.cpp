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

// Ray class methods
// Part of Retra, the Reference Tracer

#include "ray.h"

#include <iostream>
#include <cstdlib>

#include "aux.h"
#include "scene.h"

namespace Retra {

    RGB Ray::trace()
    {
        if ( RGB::Black == color || depth < 0 )
            return RGB::Black;

        if ( lightHit )
        {
            // Hit a lightsource. This path ends here
            paint( lightHit->getEmission() );
            return color;
        }
        else if ( !thingHit )
        {
            // Missed all surfaces. This path ends here
            paint( scene->getSky().color );
            return color;
        }
        paint( thingHit->getColor() );
        --depth;

        // Decide what the Surface will behave like this time
        switch ( thingHit->interact() )
        {
            case Material::DIFFUSE: // Basic Lambertian reflection
                return bounceDiffuse();
            case Material::METALLIC:// Simplified Fresnel reflection
                return bounceMetallic();
            case Material::REFLECT: // Ideal reflection (mirror)
                return bounceReflect();
            case Material::REFRACT: // Ideal dielectric refraction
                return bounceRefract();
            default:
                assert( false );
        }
    }

    bool Ray::russianRoulette()
    {
        // Russian roulette is a common heuristic for path termination
        // Here we use a variant based on current color intensity
        // A lower rrLimit keeps more paths alive
        const double maxColor = max( color.x, max(color.y, color.z) );
        if ( rrLimit <= maxColor )
            return false;
        if ( maxColor < (double)std::rand() * rrLimit / RAND_MAX )
            return true;
        color *= (rrLimit / maxColor);
        return false;
    }

    RGB Ray::bounceDiffuse()
    {
        const Vector surfaceNormal = thingPartHit->getNormal( origin );
        RGB currentColor = color * scene->getDirectLight( origin, surfaceNormal );
        if ( depth < 1 || russianRoulette() )
            return currentColor;
        direction = Vector::random( surfaceNormal );
        paint( RGB::White * (direction * surfaceNormal) );
        traceToNextIntersection();
        return currentColor + trace();
    }

    RGB Ray::bounceMetallic()
    {
        const bool into = insideThings.empty() || insideThings.top() != thingHit;
        double n1;
        if ( insideThings.empty() )
            n1 = 1.0;
        else
            n1 = insideThings.top()->getRefractiveIndex();
        double n2;
        if ( into )
            n2 = thingHit->getRefractiveIndex();
        else
        {
            const Thing* tempTop = insideThings.top();
            insideThings.pop();
            if ( insideThings.empty() )
                n2 = 1.0;
            else
                n2 = insideThings.top()->getRefractiveIndex();
            insideThings.push( tempTop );
        }
        const Vector surfaceNormal = thingPartHit->getNormal( origin );
        direction -= surfaceNormal * (direction * surfaceNormal) * 2;
        const double cosTheta = direction * surfaceNormal;
        paint( RGB::White * schlick( n1, n2, cosTheta ) );
        if ( depth < 1 || russianRoulette() )
            return RGB::Black;
        traceToNextIntersection();
        return trace();
    }

    RGB Ray::bounceReflect()
    {
        if ( depth < 1 || russianRoulette() )
            return RGB::Black;
        const Vector surfaceNormal = thingPartHit->getNormal( origin );
        direction -= surfaceNormal * (direction * surfaceNormal) * 2;
        traceToNextIntersection();
        return trace();
    }

    RGB Ray::bounceRefract()
    {
        // en.wikipedia.org/wiki/Snell's_law
        // http://graphics.stanford.edu/courses/cs148-10-summer/docs/2006--degreve--reflection_refraction.pdf
        if ( depth < 1 || russianRoulette() )
            return RGB::Black;
        const bool into = insideThings.empty() || insideThings.top() != thingHit;
        double n1;
        if ( insideThings.empty() )
            n1 = 1.0; // Vacuum
        else
            n1 = insideThings.top()->getRefractiveIndex();
        double n2;
        if ( into )
            n2 = thingHit->getRefractiveIndex();
        else
        {
            const Thing* tempTop = insideThings.top();
            insideThings.pop();
            if ( insideThings.empty() )
                n2 = 1.0; // Vacuum
            else
                n2 = insideThings.top()->getRefractiveIndex();
            insideThings.push( tempTop );
        }
        const double eta = n1 / n2;
        const Vector surfaceNormal = thingPartHit->getNormal( origin );
        const double cosTheta1 = abs( direction * surfaceNormal );
        const double sinTheta2Squared = eta * eta * ( 1.0 - cosTheta1 * cosTheta1 ); // sin(x)^2 + cos(x)^2 == 1
        if ( 1 < sinTheta2Squared )
        {
            // Total internal reflection
            direction += surfaceNormal * (direction * surfaceNormal) * 2;
        }
        else
        {
            // Actual refractive transmission
            const double cosTheta2 = sqrt( 1.0 - sinTheta2Squared );
            direction = direction * eta + surfaceNormal * ( eta * cosTheta1 - cosTheta2 ) * ( direction * surfaceNormal < 0 ? 1.0 : -1.0 );
            if ( into )
                insideThings.push( thingHit );
            else
                insideThings.pop();
        }
        traceToNextIntersection();
        return trace();
    }

    void Ray::traceToNextIntersection()
    {
        assert( 0 <= depth );

        lightHit     = NULL;
        lightPartHit = NULL;
        thingHit     = NULL;
        thingPartHit = NULL;

        double t, nearestT = INF;

        // Check foreground Surfaces
        for ( ThingIt thing = scene->thingsBegin(); thing != scene->thingsEnd(); thing++ )
            if ( (*thing)->isBackground() == false )
                for ( ThingPartIt part = (*thing)->partsBegin(); part != (*thing)->partsEnd(); part++ )
                    if ( (t = (*part)->intersect(*this)) && t < nearestT )
                    {
                        nearestT     = t;
                        thingHit     = *thing;
                        thingPartHit = *part;
                    }

        for ( LightIt light = scene->lightsBegin(); light != scene->lightsEnd(); light++ )
            if ( (*light)->isBackground() == false )
                for ( LightPartIt part = (*light)->partsBegin(); part != (*light)->partsEnd(); part++ )
                    if ( (t = (*part)->intersect(*this)) && t < nearestT )
                    {
                        nearestT     = t;
                        lightHit     = *light;
                        lightPartHit = *part;
                        thingHit     = NULL;
                        thingPartHit = NULL;
                    }

        if ( thingHit || lightHit )
        {
            origin = (*this)[nearestT];
            return;
        }

        // Check background Surfaces
        for ( ThingIt thing = scene->thingsBegin(); thing != scene->thingsEnd(); thing++ )
            if ( (*thing)->isBackground() == true )
                for ( ThingPartIt part = (*thing)->partsBegin(); part != (*thing)->partsEnd(); part++ )
                    if ( (t = (*part)->intersect(*this)) && t < nearestT )
                    {
                        nearestT     = t;
                        lightHit     = NULL;
                        lightPartHit = NULL;
                        thingHit     = *thing;
                        thingPartHit = *part;
                    }

        for ( LightIt light = scene->lightsBegin(); light != scene->lightsEnd(); light++ )
            if ( (*light)->isBackground() == true )
                for ( LightPartIt part = (*light)->partsBegin(); part != (*light)->partsEnd(); part++ )
                    if ( (t = (*part)->intersect(*this)) && t < nearestT )
                    {
                        nearestT     = t;
                        lightHit     = *light;
                        lightPartHit = *part;
                        thingHit     = NULL;
                        thingPartHit = NULL;
                    }

        origin = (*this)[nearestT];
    }

    double Ray::schlick( double n1, double n2, double cosTheta )
    {
        // http://en.wikipedia.org/wiki/Schlick%27s_approximation
        const double R0 = (n1 - n2) * (n1 - n2) / ( (n1 + n2) * (n1 + n2) );
        return R0 + (1.0 - R0) * pow(1.0 - cosTheta, 5);
    }

}

