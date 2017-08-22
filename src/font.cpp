/* Copyright (C) 2017 Coos Baakman
  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.
  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:
  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
*/

#include <cstring>
#include <string>
#include <cctype>
#include <cmath>
#include <math.h>
#include <stdlib.h>
#include <algorithm>
#include <functional>

#include "font.h"
#include "exception.h"
#include "log.h"

#define PI 3.1415926535897932384626433832795

/**
 * Parses a XML character reference from a string.
 *
 * See also: https://en.wikipedia.org/wiki/Numeric_character_reference
 *
 * Only handles strings with one unicode character in it and
 * no names, only numbers :(
 */
bool ParseXMLUnicode(const char *repr, unicode_char *out)
{
    size_t len = strlen(repr);

    if (len == 1) // ascii
    {
        *out = repr [0];
        return true;
    }
    else if (repr [0] == '&' && repr [1] == '#' && repr [len - 1] == ';' ) // html code
    {
        if (repr [2] == 'x') // hexadecimal

            sscanf(repr + 3, "%x;", out);

        else // decimal

            sscanf(repr + 2, "%d;", out);

        return true;
    }
    else if ((repr + len) == next_from_utf8 (repr, out)) // utf-8 code
    {
        return true;
    }

    // neither of the above
    return false;
}

/**
 * Parses n comma/space separated floats from the given string and
 * returns a pointer to the text after it.
 */
const char *SVGParsePathFloats(const int n, const char *text, float outs [])
{
    int i;
    for (i = 0; i < n; i++)
    {
        if (!*text)
            return NULL;

        while (isspace(*text) || *text == ',')
        {
            text++;
            if (!*text)
                return NULL;
        }

        text = ParseFloat(text, &outs[i]);
        if (!text)
            return NULL;
    }

    return text;
}
/**
 * This function, used to draw quadratic curves in cairo, is
 * based on code from cairosvg(http://cairosvg.org/)
 */
void Quadratic2Bezier(float *px1, float *py1, float *px2, float *py2, const float x, const float y)
{
    float xq1 = (*px2) * 2 / 3 + (*px1) / 3,
          yq1 = (*py2) * 2 / 3 + (*py1) / 3,
          xq2 = (*px2) * 2 / 3 + x / 3,
          yq2 = (*py2) * 2 / 3 + y / 3;

    *px1 = xq1;
    *py1 = yq1;
    *px2 = xq2;
    *py2 = yq2;
}

/**
 * Turns a cairo surface into a GL texture of equal dimensions.
 * Only works for colored surfaces with alpha channel.
 * Should only be used for generating glyph textures, was not
 * tested for other things.
 */
void Cairo2GLTex(cairo_surface_t *surface, GLuint *pTex)
{
    cairo_status_t status;

    cairo_format_t format = cairo_image_surface_get_format(surface);
    if (format != CAIRO_FORMAT_ARGB32)
    {
        throw FormatableException("Cairo2GLTex: unsupported format detected on cairo surface");
    }

    int w = cairo_image_surface_get_width(surface),
        h = cairo_image_surface_get_height(surface);

    unsigned char *pData = cairo_image_surface_get_data(surface);

    status = cairo_surface_status(surface);
    if (status != CAIRO_STATUS_SUCCESS)
    {
        throw FormatableException("error accessing cairo surface data: %s",
                                  cairo_status_to_string(status));
    }

    if (pData == NULL)
    {
        throw FormatableException("cairo_image_surface_get_data returned a NULL pointer");
    }

    glGenTextures(1, pTex);
    if (!*pTex)
    {
        throw GLException("generating GL texture for glyph", glGetError());
    }

    glBindTexture(GL_TEXTURE_2D, *pTex);

    // These settings make it look smooth:
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    // This automatically clamps texture coordinates to [0.0 -- 1.0]
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

    // fill the texture:
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8,
                 (GLsizei)w, (GLsizei)h,
                 0, GL_BGRA, GL_UNSIGNED_INT_8_8_8_8_REV, // (cairo has ARGB, but GL wants BGRA)
                 pData);

    glBindTexture(GL_TEXTURE_2D, 0);
}

/**
 * This function, used for drawing an svg arc in cairo, is
 * based on code from cairosvg(http://cairosvg.org/)
 */
void cairo_svg_arc(cairo_t *cr, float current_x, float current_y,
                    float rx, float ry,
                    const float x_axis_rotation,
                    const bool large_arc, const bool sweep,
                    float x, float y)
{
    if (rx == 0 || ry == 0) // means straight line
    {
        cairo_line_to(cr, x, y);
        return;
    }

    float radii_ratio = ry / rx,

          dx = x - current_x,
          dy = y - current_y,

          // Transform the target point from elipse to circle
          xe = dx * cos(-x_axis_rotation) - dy * sin(-x_axis_rotation),
          ye =(dy * cos(-x_axis_rotation) + dx * sin(-x_axis_rotation)) / radii_ratio,

          // angle between the line from current to target point and the x axis
          angle = atan2 (ye, xe);

    // Move the target point onto the x axis
    // The current point was already on the x-axis
    xe = sqrt(xe * xe + ye * ye);
    ye = 0.0f;

    // Update the first radius if it is too small
    rx = std::max(rx, xe / 2);

    // Find one circle centre
    float xc = xe / 2,
          yc = sqrt(rx * rx - xc * xc);

    // fix for a glitch, appearing on some machines:
    if (rx == xc)
        yc = 0.0f;

    // Use the flags to pick a circle center
    if (!(large_arc != sweep))
        yc = -yc;

    // Rotate the target point and the center back to their original circle positions

    float sin_angle = sin(angle),
          cos_angle = cos(angle);

    ye = xe * sin_angle;
    xe = xe * cos_angle;

    float ax = xc * cos_angle - yc * sin_angle,
          ay = yc * cos_angle + xc * sin_angle;
    xc = ax;
    yc = ay;

    // Find the drawing angles, from center to current and target points on circle:
    float angle1 = atan2 (0.0f - yc, 0.0f - xc), // current is shifted to 0,0
          angle2 = atan2 (  ye - yc,   xe - xc);

    cairo_save(cr);
    cairo_translate(cr, current_x, current_y);
    cairo_rotate(cr, x_axis_rotation);
    cairo_scale(cr, 1.0f, radii_ratio);

    if (sweep)
        cairo_arc(cr, xc, yc, rx, angle1, angle2);
    else
        cairo_arc_negative(cr, xc, yc, rx, angle1, angle2);

    cairo_restore(cr);
}

void InputSVGPath(const char *d, cairo_t *cr)
{
    const char *nd;
    float f[6],
          x1, y1, x2, y2,
          x = 0, y = 0,
          qx1, qy1, qx2, qy2;

    char prev_symbol, symbol = 'm';
    bool upper = false;

    while (*d)
    {
        prev_symbol = symbol;

        // Get the next path symbol:

        while (isspace(*d))
            d++;

        upper = isupper(*d); // upper is absolute, lower is relative
        symbol = tolower(*d);
        d++;

        // Take the approriate action for the symbol:
        switch (symbol)
        {
        case 'z': // closepath

            cairo_close_path(cr);
            break;

        case 'm': // moveto(x y)+

            while ((nd = SVGParsePathFloats(2, d, f)))
            {
                if (upper)
                {
                    x = f[0];
                    y = f[1];
                }
                else
                {
                    x += f[0];
                    y += f[1];
                }

                cairo_move_to(cr, x, y);

                d = nd;
            }
            break;

        case 'l': // lineto(x y)+

            while ((nd = SVGParsePathFloats(2, d, f)))
            {
                if (upper)
                {
                    x = f[0];
                    y = f[1];
                }
                else
                {
                    x += f[0];
                    y += f[1];
                }

                cairo_line_to(cr, x, y);

                d = nd;
            }
            break;

        case 'h': // horizontal lineto x+

            while ((nd = SVGParsePathFloats(1, d, f)))
            {
                if (upper)
                {
                    x = f[0];
                }
                else
                {
                    x += f[0];
                }

                cairo_line_to(cr, x, y);

                d = nd;
            }
            break;

        case 'v': // vertical lineto y+

            while ((nd = SVGParsePathFloats(1, d, f)))
            {
                if (upper)
                {
                    y = f[0];
                }
                else
                {
                    y += f[0];
                }

                cairo_line_to(cr, x, y);

                d = nd;
            }
            break;

        case 'c': // curveto(x1 y1 x2 y2 x y)+

            while ((nd = SVGParsePathFloats(6, d, f)))
            {
                if (upper)
                {
                    x1 = f[0]; y1 = f[1];
                    x2 = f[2]; y2 = f[3];
                    x = f[4]; y = f[5];
                }
                else
                {
                    x1 = x + f[0]; y1 = y + f[1];
                    x2 = x + f[2]; y2 = y + f[3];
                    x += f[4]; y += f[5];
                }

                cairo_curve_to(cr, x1, y1, x2, y2, x, y);

                d = nd;
            }
            break;

        case 's': // shorthand/smooth curveto(x2 y2 x y)+

            while ((nd = SVGParsePathFloats(4, d, f)))
            {
                if (prev_symbol == 's' || prev_symbol == 'c')
                {
                    x1 = x + (x - x2); y1 = y + (y - y2);
                }
                else
                {
                    x1 = x; y1 = y;
                }
                prev_symbol = symbol;

                if (upper)
                {
                    x2 = f[0]; y2 = f[1];
                    x = f[2]; y = f[3];
                }
                else
                {
                    x2 = x + f[0]; y2 = y + f[1];
                    x += f[2]; y += f[3];
                }

                cairo_curve_to(cr, x1, y1, x2, y2, x, y);

                d = nd;
            }
            break;

        case 'q': // quadratic Bezier curveto(x1 y1 x y)+

            while ((nd = SVGParsePathFloats(4, d, f)))
            {
                x1 = x; y1 = y;
                if (upper)
                {
                    x2 = f[0]; y2 = f[1];
                    x = f[2]; y = f[3];
                }
                else
                {
                    x2 = x + f[0]; y2 = y + f[1];
                    x += f[2]; y += f[3];
                }

                // Cairo doesn't do quadratic, so fake it:

                qx1 = x1; qy1 = y1; qx2 = x2; qy2 = y2;
                Quadratic2Bezier(&qx1, &qy1, &qx2, &qy2, x, y);
                cairo_curve_to(cr, qx1, qy1, qx2, qy2, x, y);

                d = nd;
            }
            break;

        case 't': // Shorthand/smooth quadratic Bézier curveto(x y)+

            while ((nd = SVGParsePathFloats(2, d, f)))
            {
                x1 = x; y1 = y;
                if (prev_symbol == 't' || prev_symbol == 'q')
                {
                    x2 = x + (x - x2); y2 = y + (y - y2);
                }
                else
                {
                    x2 = x; y2 = y;
                }

                if (upper)
                {
                    x = f[0]; y = f[1];
                }
                else
                {
                    x += f[0]; y += f[1];
                }

                // Cairo doesn't do quadratic, so fake it:

                qx1 = x1; qy1 = y1; qx2 = x2; qy2 = y2;
                Quadratic2Bezier(&qx1, &qy1, &qx2, &qy2, x, y);
                cairo_curve_to(cr, qx1, qy1, qx2, qy2, x, y);

                d = nd;
            }
            break;

        case 'a': // elliptical arc(rx ry x-axis-rotation large-arc-flag sweep-flag x y)+

            while (isspace(*d))
                d++;

            while (isdigit(*d) || *d == '-')
            {
                nd = SVGParsePathFloats(3, d, f);
                if (!nd)
                {
                    throw FormatableException("arc incomplete. Missing first 3 floats in %s", d);
                }

                float rx = f[0], ry = f[1],
                      a = f[2] * PI / 180; // convert degrees to radians

                while (isspace(*nd) || *nd == ',')
                    nd++;

                if (!isdigit(*nd))
                {
                    throw FormatableException("arc incomplete. Missing large arc digit in %s", d);
                }

                bool large_arc = *nd != '0';

                do {
                    nd++;
                } while (isspace(*nd) || *nd == ',');

                if (!isdigit(*nd))
                {
                    throw FormatableException("arc incomplete. Missing sweep digit in %s", d);
                }

                bool sweep = *nd != '0';

                do
                {
                    nd++;
                }
                while (isspace(*nd) || *nd == ',');

                nd = SVGParsePathFloats(2, nd, f);
                if (!nd)
                {
                    throw FormatableException("arc incomplete. Missing last two floats in %s", d);
                }

                while (isspace(*nd))
                    nd++;

                d = nd;

                // parsing round done, now draw the elipse

                // starting points:
                x1 = x; y1 = y;

                // end points:
                if (upper)
                {
                    x = f[0];
                    y = f[1];
                }
                else
                {
                    x += f[0];
                    y += f[1];
                }

                cairo_svg_arc(cr, x1, y1,
                              rx, ry,
                              a,
                              large_arc, sweep,
                              x, y);
            }  // end loop

            break;
        }
    }
}

/**
 * Computes a new value for the bounding box, based on the shape of the glyph.
 * Bounding box is in glyph coordinates.
 */
void GetGlyphExtents(const char *d, BBox *pBox)
{
    cairo_status_t status;
    cairo_surface_t *surface;
    cairo_t *cr;
    double x1, y1, x2, y2, w, h;

    surface = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, 0, 0);
    status = cairo_surface_status(surface);
    if (status != CAIRO_STATUS_SUCCESS)
    {
        throw FormatableException("error creating cairo 0 x 0 test surface: %s",
                                  cairo_status_to_string(status));
    }

    cr = cairo_create(surface);

    try
    {
        InputSVGPath(d, cr);
    }
    catch (std::exception &e)
    {
        // Free these first:
        cairo_surface_destroy(surface);
        cairo_destroy(cr);

        throw e;
    }

    cairo_path_extents(cr, &x1, &y1, &x2, &y2);
    if (x1 == 0.0 && y1 == 0.0 && x2 == 0.0 && y2 == 0.0)
    {
        // Free these first:
        cairo_surface_destroy(surface);
        cairo_destroy(cr);

        throw FormatableException("cairo_path_extents returned 0 0 0 0");
    }

    w = x2 - x1;
    h = y2 - y1;

    pBox->left = (GLfloat)x1 - w;
    pBox->bottom = (GLfloat)y1 - h;
    pBox->right = (GLfloat)x2 + w;
    pBox->top = (GLfloat)y2 + h;

    cairo_surface_destroy(surface);
    cairo_destroy(cr);
}

// This is used only when clearing the background in cairo:
#define GLYPH_BBOX_MARGIN 10.0f

/**
 * Parses the glyph path and creates its texture.
 * (http://www.w3.org/TR/SVG/paths.html)
 *
 * :param pGlyph: glyph object, whose texture must be created.
 * :param pBox: font's bounding box
 * :param d: path data as in svg
 * :param multiply: multiplies the size of the glyph
 * :returns: true on success, falso on error
 */
void MakeGlyphTexture(const char *d, const BBox *pBox, const float multiply, Glyph *pGlyph, GlyphFillFunc glyphFunc)
{
    cairo_status_t status;
    cairo_surface_t *surface;
    cairo_t *cr;

    BBox glyphBBOX = *pBox;
    GetGlyphExtents(d, &glyphBBOX);

    /* Cairo surfaces and OpenGL textures have integer dimensions, but
     * glyph bounding boxes consist floating points. Make sure the bounding box fits onto the texture:
     */
    int tex_w = (int)ceil(multiply * (glyphBBOX.right - glyphBBOX.left)),
        tex_h = (int)ceil(multiply * (glyphBBOX.top - glyphBBOX.bottom));

    GLfloat bh = pBox->top - pBox->bottom;

    if (tex_w <= 0 || tex_h <= 0)
    {
        throw FormatableException("dimensions %d x %d computed for glyph texture",
                                  tex_w, tex_h);
    }

    pGlyph->tex = NULL;

    // bounding box is already multiplied at this point
    surface = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, tex_w, tex_h);
    status = cairo_surface_status(surface);
    if (status != CAIRO_STATUS_SUCCESS)
    {
        throw FormatableException("error creating cairo surface for glyph: %s",
                                  cairo_status_to_string(status));
    }

    cr = cairo_create(surface);

    // within the cairo surface, move to the glyph's coordinate system
    cairo_save(cr);
    cairo_scale(cr, multiply, multiply);
    cairo_translate(cr, -glyphBBOX.left, -glyphBBOX.bottom);

    try
    {
        InputSVGPath(d, cr);
    }
    catch (std::exception &e)
    {
        // Free these first:
        cairo_surface_destroy(surface);
        cairo_destroy(cr);

        throw e;
    }

    // Do a restore to make the lines have the correct thickness:
    cairo_restore(cr);

    // Fill it in
    glyphFunc(cr);

    // Convert the cairo surface to a GL texture:
    cairo_surface_flush(surface);
    try
    {
        Cairo2GLTex(surface, &pGlyph->tex);
    }
    catch (std::exception &e)
    {
        // Free these first:
        cairo_surface_destroy(surface);
        cairo_destroy(cr);

        throw e;
    }

    // Don't need this anymore:
    cairo_surface_destroy(surface);
    cairo_destroy(cr);

    pGlyph->tex_w = tex_w;
    pGlyph->tex_h = tex_h;
    pGlyph->tex_px = -glyphBBOX.left * multiply;
    pGlyph->tex_py = glyphBBOX.top * multiply;
}


/**
 * The header contains important settings
 * :param pFnt: xml font tag
 * :param pFont: font object to set data to
 * :returns: true on success, false when data is missing
 */
void ParseSvgFontHeader(const xmlNodePtr pFnt, Font *pFont)
{
    xmlChar *pAttrib;
    xmlNodePtr pFace = NULL,
               pChild;

    for (pChild = pFnt->children; pChild; pChild = pChild -> next)
    {
        if (StrCaseCompare((const char *) pChild->name, "font-face") == 0) {

            pFace = pChild;
            break;
        }
    }
    if (!pFace)
    {
        throw FormatableException("cannot find a font-face tag, it\'s required");
    }

    pAttrib = xmlGetProp(pFace, (const xmlChar *)"units-per-em");
    if (!pAttrib)
    {
        // other size specifications are not implemented here !

        throw FormatableException("units-per-em is not set on font-face tag, it\'s required");
    }
    pFont->unitsPerEM = atoi((const char *)pAttrib);
    xmlFree(pAttrib);

    float multiply = pFont->outputSize / pFont->unitsPerEM;

    pAttrib = xmlGetProp(pFace, (const xmlChar *)"bbox");
    if (!pAttrib)
    {
        // other size specifications are not implemented here !

        throw FormatableException("bbox is not set on font-face tag, it\'s required");
    }

    if (!SVGParsePathFloats(4, (const char *)pAttrib, (float *)&(pFont->inputBBOX)))
    {
        xmlFree(pAttrib);

        throw FormatableException("unable to parse 4 bbox numbers from \'%s\'\n",
                                  (const char *)pAttrib);
    }
    xmlFree(pAttrib);

    /*
        horiz-adv-x, horiz-origin-x & horiz-origin-y are optional.
        If not set, they are presumed 0.
     */

    pAttrib = xmlGetProp(pFnt, (const xmlChar *)"horiz-adv-x");
    if (pAttrib)
    {
        pFont->output_horiz_adv_x = atoi((const char *)pAttrib) * multiply;
        xmlFree(pAttrib);
    }
    else
        pFont->output_horiz_adv_x = 0;

    pAttrib = xmlGetProp(pFnt, (const xmlChar *)"horiz-origin-x");
    if (pAttrib)
    {
        pFont->output_horiz_origin_x = atoi((const char *)pAttrib) * multiply;
        xmlFree(pAttrib);
    }
    else
        pFont->output_horiz_origin_x = 0;

    pAttrib = xmlGetProp(pFnt, (const xmlChar *)"horiz-origin-y");
    if (pAttrib)
    {
        pFont->output_horiz_origin_y = atoi((const char *)pAttrib) * multiply;
        xmlFree(pAttrib);
    }
    else
        pFont->output_horiz_origin_y = 0;
}

void ClearFont(Font *pFont)
{
    for (auto it : pFont->glyphs)
    {
        glDeleteTextures(1, &(it.second.tex));
        it.second.tex = NULL;
    }
}

#define DEFAULT_CHAR '?'

void MakeSVGFont(const xmlDocPtr pDoc, const int size, GlyphFillFunc glyphFunc, Font *pFont)
{
    if (pDoc == NULL)
    {
        throw FormatableException("null pointer for xml document");
    }

    pFont->outputSize = size;

    // The root tag of the xml document is svg:
    bool success;
    xmlNodePtr pRoot = xmlDocGetRootElement(pDoc);
    if (!pRoot)
    {
        throw FormatableException("no root element in svg doc");
    }

    if (StrCaseCompare((const char *) pRoot->name, "svg") != 0) {

        throw FormatableException(
            "document of the wrong type, found \'%s\' instead of \'svg\'",
            pRoot->name);
    }

    // In svg, fonts are found in the defs section.

    xmlNode *pDefs = NULL,
            *pFnt = NULL,
            *pChild = pRoot->children;

    while (pChild) {
        if (StrCaseCompare((const char *) pChild->name, "defs") == 0) {

            pDefs = pChild;
            break;
        }
        pChild = pChild->next;
    }

    if (!pDefs)
    {
        throw FormatableException(
            "no defs element found in svg doc, expecting font to be there\n");
    }

    // Pick the first font tag we see:

    pChild = pDefs->children;
    while (pChild) {
        if (StrCaseCompare((const char *) pChild->name, "font") == 0) {

            pFnt = pChild;
            break;
        }
        pChild = pChild->next;
    }

    if (!pFnt)
    {
        throw FormatableException(
            "no font element found in defs section, expecting it to be there\n");
    }

    // Get data from the font header:
    ParseSvgFontHeader(pFnt, pFont);

    float multiply = pFont->outputSize / pFont->unitsPerEM;

    /*
        Confusing!
        In this font format, the same name can be on multiple glyphs.
        One glyph name can refer to more than one unicode entry.
        Example: 'A' referring to both the Roman and Cyrillic A
     */
    std::map<std::string, std::list <unicode_char>> glyph_name_lookup;

    // iterate over glyph tags
    for (pChild = pFnt->children; pChild; pChild = pChild->next)
    {
        if (StrCaseCompare((const char *)pChild->name, "glyph") == 0)
        {
            xmlChar *pAttrib;
            unicode_char ch;

            pAttrib = xmlGetProp(pChild, (const xmlChar *)"unicode");
            if (!pAttrib)
            {
                /* A glyph without an unicode id,
                   Pretty useless glyph this is :(
                 */
                continue;
            }

            if (!ParseXMLUnicode((const char *)pAttrib, &ch))
            {
                xmlFree(pAttrib);

                throw FormatableException(
                           "failed to interpret unicode id \"%s\"",
                           (const char *)pAttrib);
            }
            xmlFree(pAttrib);

            pAttrib = xmlGetProp(pChild, (const xmlChar *)"glyph-name");
            if (pAttrib)
            {
                /*
                    In some fonts, glyphs are given names to identify them with.
                    These names need to be remembered during the parsing process.
                 */

                std::string name = std::string((const char *)pAttrib);
                xmlFree(pAttrib);

                if (glyph_name_lookup.find(name) == glyph_name_lookup.end())
                    glyph_name_lookup [name] = std::list <unicode_char> ();
                glyph_name_lookup [name].push_back(ch);
            }

            if (pFont->glyphs.find(ch) != pFont->glyphs.end())
            {
                // means the table already has an entry for this unicode character.

                throw FormatableException(
                           "error: duplicate unicode id 0x%X", ch);
            }

            // Add new entry to glyphs table
            pFont->glyphs[ch] = Glyph();
            pFont->glyphs[ch].ch = ch;

            // Get path data and generate a glyph texture from it:
            pAttrib = xmlGetProp(pChild, (const xmlChar *)"d");

            if (pAttrib)
            {
                try
                {
                    MakeGlyphTexture((const char *)pAttrib,
                                     &(pFont->inputBBOX), multiply,
                                     &pFont->glyphs[ch], glyphFunc);
                }
                catch (std::exception &e)
                {
                    // Free this first:
                    xmlFree(pAttrib);

                    throw e;
                }
            }
            else // path may be an empty string, whitespace for example
            {
                pFont->glyphs[ch].tex = NULL;
            }
            xmlFree(pAttrib);

            /*
                horiz-adv-x, horiz-origin-x & horiz-origin-y are optional,
                if not present, use the font's default setting.
             */

            pAttrib = xmlGetProp(pChild, (const xmlChar *)"horiz-adv-x");
            if (pAttrib)

                pFont->glyphs[ch].output_horiz_adv_x = atoi((const char *)pAttrib) * multiply;
            else
                pFont->glyphs[ch].output_horiz_adv_x = -1.0f;

            xmlFree(pAttrib);

            pAttrib = xmlGetProp(pChild, (const xmlChar *)"horiz-origin-x");
            if (pAttrib)

                pFont->glyphs[ch].output_horiz_origin_x = atoi((const char *)pAttrib) * multiply;
            else
                pFont->glyphs[ch].output_horiz_origin_x = -1.0f;

            xmlFree(pAttrib);

            pAttrib = xmlGetProp(pChild, (const xmlChar *)"horiz-origin-y");
            if (pAttrib)

                pFont->glyphs[ch].output_horiz_origin_y = atoi((const char *)pAttrib) * multiply;
            else
                pFont->glyphs[ch].output_horiz_origin_y = -1.0f;

            xmlFree(pAttrib);
        }
    }

    // Now look for horizontal kerning data, iterate over hkern tags:
    for (pChild = pFnt->children; pChild; pChild = pChild->next)
    {
        if (StrCaseCompare((const char *)pChild->name, "hkern") == 0)
        {
            xmlChar *pAttrib;
            std::list<std::string> g1, g2, su1, su2;
            std::list<unicode_char> u1, u2;

            // Get kern value first:
            pAttrib = xmlGetProp(pChild, (const xmlChar *)"k");
            if (!pAttrib)
            {
                throw FormatableException(
                           "One of the hkern tags misses its \'k\' value");
            }
            float k = atoi((const char *)pAttrib) * multiply;
            xmlFree(pAttrib);

            /*
                Now determine the kern pair. In svg a kern pair can
                be a many-to-many, but we store them as
                one-to-one glyph kern pairs.
                u1 & u2 specify unicode references, that need to be parsed.
                g1 & g2 specify glyph names, that need to be looked up.
             */

            pAttrib = xmlGetProp(pChild, (const xmlChar *)"g1");
            if (pAttrib)
                split((const char *)pAttrib, ',', g1);
            xmlFree(pAttrib);

            pAttrib = xmlGetProp(pChild, (const xmlChar *)"u1");
            if (pAttrib)
                split((const char *)pAttrib, ',', su1);
            xmlFree(pAttrib);

            pAttrib = xmlGetProp(pChild, (const xmlChar *)"g2");
            if (pAttrib)
                split((const char *)pAttrib, ',', g2);
            xmlFree(pAttrib);

            pAttrib = xmlGetProp(pChild, (const xmlChar *)"u2");
            if (pAttrib)
                split((const char *)pAttrib, ',', su2);
            xmlFree(pAttrib);

            if (g1.size() == 0 && su1.size() == 0)
            {
                throw FormatableException(
                           "no first group found on one of the hkern entries");
            }

            if (g2.size() == 0 && su2.size() == 0)
            {
                throw FormatableException(
                           "no second group found on one of the hkern entries");
            }

            // Combine g1 & su1 to make u1
            // Combine g2 & su2 to make u2

            for (const std::string name : g1)
            {
                if (glyph_name_lookup.find(name) == glyph_name_lookup.end())
                {
                    throw FormatableException(
                               "undefined glyph name: %s", name.c_str());
                }
                for (const unicode_char ch : glyph_name_lookup.at(name))
                    u1.push_back(ch);
            }
            for (const std::string name : g2)
            {
                if (glyph_name_lookup.find(name) == glyph_name_lookup.end())
                {
                    throw FormatableException(
                               "undefined glyph name: %s", name.c_str());
                }
                for (const unicode_char ch : glyph_name_lookup.at(name))
                    u2.push_back(ch);
            }
            for (std::list<std::string>::const_iterator it = su1.begin(); it != su1.end(); it++)
            {
                std::string s = *it;
                unicode_char c;

                if (!ParseXMLUnicode(s.c_str(), &c))
                {
                    throw FormatableException(
                               "unparsable unicode char: %s", s.c_str());
                }
                u1.push_back(c);
            }
            for (std::list<std::string>::const_iterator it = su2.begin(); it != su2.end(); it++)
            {
                std::string s = *it;
                unicode_char c;

                if (!ParseXMLUnicode(s.c_str(), &c))
                {
                    throw FormatableException(
                               "unparsable unicode char: %s", s.c_str());
                }
                u2.push_back(c);
            }

            // Add the combinations from u1 and u2 to the kern table

            for (std::list<unicode_char>::const_iterator it = u1.begin(); it != u1.end(); it++)
            {
                unicode_char c1 = *it;

                if (pFont->outputHorizontalKernTable.find(c1) == pFont->outputHorizontalKernTable.end())
                {
                    pFont->outputHorizontalKernTable[c1] = std::map<unicode_char, float>();
                }

                for (std::list<unicode_char>::const_iterator jt = u2.begin(); jt != u2.end(); jt++)
                {
                    unicode_char c2 = *jt;

                    pFont->outputHorizontalKernTable[c1][c2] = k;
                }
            }
        }
    }

    if (pFont->glyphs.find(DEFAULT_CHAR) == pFont->glyphs.end())
    {
        // even this is missing ><

        pFont->glyphs[DEFAULT_CHAR] = Glyph();
        pFont->glyphs[DEFAULT_CHAR].tex = NULL;
        pFont->glyphs[DEFAULT_CHAR].output_horiz_origin_x = 0;
        pFont->glyphs[DEFAULT_CHAR].output_horiz_origin_y = 0;
        pFont->glyphs[DEFAULT_CHAR].output_horiz_adv_x = 0;
        pFont->glyphs[DEFAULT_CHAR].tex_w = (pFont->inputBBOX.right - pFont->inputBBOX.left) * multiply;
        pFont->glyphs[DEFAULT_CHAR].tex_h = (pFont->inputBBOX.top - pFont->inputBBOX.bottom) * multiply;
        pFont->glyphs[DEFAULT_CHAR].tex_px = -pFont->inputBBOX.left * multiply;
        pFont->glyphs[DEFAULT_CHAR].tex_py = -pFont->inputBBOX.bottom * multiply;
    }
}

/**
 * Renders one glyph at x,y
 */
void glRenderGlyph(const Font *pFont, const Glyph *pGlyph, const GLfloat x, const GLfloat y)
{
    glTranslatef(-pGlyph->tex_px, -pGlyph->tex_py, 0.0f);

    // Render the glyph texture to a bbox-sized quad at x,y:
    glBindTexture(GL_TEXTURE_2D, pGlyph->tex);

    glBegin(GL_QUADS);

    glTexCoord2f(0.0f, 1.0f);
    glVertex2f(x, y);

    glTexCoord2f(1.0f, 1.0f);
    glVertex2f(x + pGlyph->tex_w, y);

    glTexCoord2f(1.0f, 0.0f);
    glVertex2f(x + pGlyph->tex_w, y + pGlyph->tex_h);

    glTexCoord2f(0.0f, 0.0f);
    glVertex2f(x, y + pGlyph->tex_h);

    glEnd();

    glTranslatef(pGlyph->tex_px, pGlyph->tex_py, 0.0f);
}
/**
 * looks up the kern value for two characters: prev_c and c
 */
float GetHKern(const Font *pFont, const unicode_char prev_c, const unicode_char c)
{
    if (prev_c && pFont->outputHorizontalKernTable.find(prev_c) != pFont->outputHorizontalKernTable.end())
    {
        const std::map<unicode_char, float> row = pFont->outputHorizontalKernTable.at(prev_c);
        if (row.find(c) != row.end())
        {
            return row.at(c);
        }
    }
    return 0.0f;
}
/**
 * Looks up the horizontal advance value for a glyph.
 */
float GetHAdv(const Font *pFont, const Glyph *pGlyph)
{
    if (pGlyph && pGlyph->output_horiz_adv_x > 0)

        return pGlyph->output_horiz_adv_x;
    else
        return pFont->output_horiz_adv_x; // default value
}
/**
 * Gets the width of the first word in pUTF8.
 */
float NextWordWidth(const Font *pFont, const char *pUTF8, unicode_char prev_c)
{
    float w = 0.0f;
    unicode_char c = prev_c;
    const Glyph *pGlyph;
    const char *nUTF8;

    // First include the spaces before the word
    while (*pUTF8)
    {
        prev_c = c;
        nUTF8 = next_from_utf8(pUTF8, &c);

        if (isspace(c))
        {
            if (pFont->glyphs.find(c) == pFont->glyphs.end())
            {
                LOG_ERROR("0x%X, no such glyph", c);
                c = DEFAULT_CHAR;
            }

            pGlyph = &pFont->glyphs.at(c);

            w += -GetHKern(pFont, prev_c, c) + GetHAdv(pFont, pGlyph);

            pUTF8 = nUTF8;
        }
        else
            break;
    }

    while (*pUTF8)
    {
        prev_c = c;
        nUTF8 = next_from_utf8(pUTF8, &c);

        if (isspace(c))
            break;
        else
        {
            if (pFont->glyphs.find(c) == pFont->glyphs.end())
            {
                LOG_ERROR("0x%X, no such glyph", c);
                c = DEFAULT_CHAR;
            }

            pGlyph = &pFont->glyphs.at(c);

            w += -GetHKern(pFont, prev_c, c) + GetHAdv(pFont, pGlyph);

            pUTF8 = nUTF8;
        }
    }

    return w;

    // now read till the next space
}
GLfloat GetLineSpacing(const Font *pFont)
{
    float multiply = pFont->outputSize / pFont->unitsPerEM;

    return multiply * (pFont->inputBBOX.top - pFont->inputBBOX.bottom);
}

/**
 * returns: true if the next word exceeds maxWidth when started at current_x, false otherwise.
 */
bool NeedNewLine(const Font *pFont, const unicode_char prev_c, const char *pUTF8, const float current_x, const float maxWidth)
{
    unicode_char c;
    next_from_utf8(pUTF8, &c);
    const Glyph *pGlyph;
    float adv;

    if (c == '\n' || c == '\r')
    {
        return true;
    }

    if (maxWidth > 0.0f)
    {
        // Maybe the next char starts a new word that doesn't fit onto this line:
        if (isspace(prev_c) && !isspace(c) && current_x > 0 && maxWidth > 0 && maxWidth < (current_x + NextWordWidth(pFont, pUTF8, prev_c)))
        {
            return true;
        }

        if (pFont->glyphs.find(c) == pFont->glyphs.end())
        {
            LOG_ERROR("0x%X, no such glyph", c);
            c = DEFAULT_CHAR;
        }
        pGlyph = &pFont->glyphs.at(c);

        adv = GetHAdv(pFont, pGlyph);

        if (current_x + adv > maxWidth)
        { // the word is so long that it doesn't fit on one line
            return true;
        }
    }

    return false;
}
/**
 * consumes characters from pUTF8 that shouldn't be taken to the next line
 * :returns: pointer to the next line
 * :param n_removed: optional, tells how many characters were removed.
 */
const char *CleanLineEnd(const char *pUTF8, int *n_removed = NULL)
{
    unicode_char c;
    const char *nUTF8;

    int n = 0;

    while (true)
    {
        nUTF8 = next_from_utf8(pUTF8, &c);

        if (c == '\n' || c == '\r')
        {
            if (n_removed)
                *n_removed = n + 1;

            // this will be the last character we take
            return nUTF8;
        }
        else if (isspace(c))
        {
            n++;
            pUTF8 = nUTF8; // go to next
        }
        else // Non-whitespace character, must not be taken away!
        {
            if (n_removed)
                *n_removed = n;

            return pUTF8;
        }
    }
}
/*
 * Looks up the origin for a glyph.
 */
void GetGlyphOrigin(const Font *pFont, const Glyph *pGlyph, float &ori_x, float &ori_y)
{
    ori_x = pFont->output_horiz_origin_x;
    ori_y = pFont->output_horiz_origin_y;

    if (pGlyph && pGlyph->output_horiz_origin_x > 0)
        ori_x = pGlyph->output_horiz_origin_x;
    if (pGlyph && pGlyph->output_horiz_origin_y > 0)
        ori_y = pGlyph->output_horiz_origin_y;
}
/**
 * Computes the width of the next line at pUTF8.
 * (how many glyphs fit in)
 * It can never be longer than maxLineWidth.
 */
float NextLineWidth(const Font *pFont, const char *pUTF8, const float maxLineWidth)
{
    float x = 0.0f, w = 0.0f;
    unicode_char c = NULL, prev_c = NULL;
    const Glyph *pGlyph;

    while (*pUTF8)
    {
        prev_c = c;

        if (NeedNewLine(pFont, prev_c, pUTF8, x, maxLineWidth))
        {
            return w;
        }

        pUTF8 = next_from_utf8(pUTF8, &c);

        if (pFont->glyphs.find(c) == pFont->glyphs.end())
        {
            LOG_ERROR("0x%X, no such glyph", c);
            c = DEFAULT_CHAR;
        }
        pGlyph = &pFont->glyphs.at(c);

        if (x > 0.0f)
            x -= GetHKern(pFont, prev_c, c);

        x += GetHAdv(pFont, pGlyph);

        if (!isspace(c))
            w = x;
    }

    return w;
}
/**
 * ThroughTextGlyphFuncs are callbacks that will be called for each glyph
 * that ThroughText encounters on its pass through the text.
 *
 * A ThroughTextGlyphFunc should return true if it wants the next glyph.
 * Glyph pointer is NULL for the terminating null character!
 *
 * x,y are the position where the glyph's origin should be.
 * string_pos is the index of the character, starting from 0
 * the last argument is the object inserted as pObject in ThroughText.
 */
typedef std::function <bool(const Glyph*, const float x, const float y, const int string_pos)> ThroughTextGlyphFunc;

/*
 * ThroughText passes through the given utf8 string with given text formatting.
 * It calls the given ThroughTextGlyphFunc for every glyph it encounters.
 * pObject can be any object that must be passed on to the ThroughTextGlyphFunc
 * in each call.
 */
void ThroughText(const Font *pFont, const char *pUTF8,
                  ThroughTextGlyphFunc GlyphFunc,
                  const int align, float maxWidth)
{
    int i = 0, n, halign = align & 0x0f;

    float x = 0.0f, y = 0.0f,
          lineWidth = NextLineWidth(pFont, pUTF8, maxWidth),
          glyph_x;

    unicode_char c = NULL,
                 prev_c = NULL;

    const Glyph *pGlyph;

    while (*pUTF8)
    {
        prev_c = c;

        // Before moving on to the next glyph, see if we need to start a new line
        if (NeedNewLine(pFont, prev_c, pUTF8, x, maxWidth))
        {
            // start new line
            pUTF8 = CleanLineEnd(pUTF8, &n);
            i += n;
            prev_c = c = NULL;

            lineWidth = NextLineWidth(pFont, pUTF8, maxWidth);
            x = 0.0f;
            y += GetLineSpacing(pFont);

            continue;
        }

        // Get next utf8 char as c
        pUTF8 = next_from_utf8(pUTF8, &c);

        if (pFont->glyphs.find(c) == pFont->glyphs.end())
        {
            LOG_ERROR("0x%X, no such glyph", c);
            c = DEFAULT_CHAR;
        }
        pGlyph = &pFont->glyphs.at(c);

        // Look up horizontal kern value:
        if (x > 0.0f)
            x -= GetHKern(pFont, prev_c, c);

        // move the glyph to the left if alignment is mid or right
        glyph_x = x;
        if (halign == TEXTALIGN_MID)

            glyph_x = x - lineWidth / 2;

        else if (halign == TEXTALIGN_RIGHT)

            glyph_x = x - lineWidth;

        // Call the given ThroughTextGlyphFunc and break out if it returns false
        if (!GlyphFunc(pGlyph, glyph_x, y, i))
            return;

        // move on to the x after the glyph and update the index:
        x += GetHAdv(pFont, pGlyph);
        i ++;
    }

    // move the glyph to the left if alignment is mid or right
    glyph_x = x;
    if (halign == TEXTALIGN_MID)

        glyph_x = x - lineWidth / 2;

    else if (halign == TEXTALIGN_RIGHT)

        glyph_x = x - lineWidth;

    // Call GlyphFunc for the terminating null, with the rightmost x value:
    GlyphFunc(NULL, glyph_x, y, i);
}
void glRenderText(const Font *pFont, const char *pUTF8, const int align, float maxWidth)
{
    glFrontFace(GL_CW);
    glActiveTexture(GL_TEXTURE0);
    glEnable(GL_TEXTURE_2D);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    ThroughText(pFont, pUTF8,
        [&] (const Glyph *pGlyph, const float x, const float y, const int string_pos)
        {
            if (!pGlyph)
                return true;

            // Render the glyph with its origin at the given x,y

            float ori_x, ori_y;
            GetGlyphOrigin(pFont, pGlyph, ori_x, ori_y);

            if (pGlyph->tex)
                glRenderGlyph(pFont, pGlyph, x - ori_x, y - ori_y);

            return true;
        },
        align, maxWidth);
}
void glRenderTextAsRects(const Font *pFont, const char *pUTF8,
                          const int from, const int to,
                          const int align, float maxWidth)
{
    if (from >= to || from < 0)
        return; // nothing to do

    float start_x, current_y, end_x;

    glFrontFace(GL_CW);
    glActiveTexture(GL_TEXTURE0);

    ThroughText(pFont, pUTF8,
        [&] (const Glyph *pGlyph, const float x, const float y, const int string_pos)
        {
            if (string_pos < from)
            {
                return true; // no rects yet, wait till after 'from'
            }

            float ori_x, ori_y;
            GetGlyphOrigin(pFont, pGlyph, ori_x, ori_y);

            const GLfloat w = GetHAdv(pFont, pGlyph),
                          h = GetLineSpacing(pFont);

            bool end_render = string_pos >= to,
                 first_glyph = string_pos <= from,

                 // If the y value has changed, means a new line started
                 new_line = abs((y - ori_y) - current_y) >= h,

                 // A rect ends when we've passed 'to' or a new line is started
                 end_prev_rect = end_render || new_line,

                 // A rect starts when we're at the first glyph or a new line is started
                 start_new_rect = new_line || first_glyph;

            if (end_prev_rect)
            {
                // Render a quad from start to previous glyph

                GLfloat x1, y1, x2, y2;

                x1 = start_x;
                x2 = end_x;
                y1 = current_y;
                y2 = y1 + h;

                glBegin(GL_QUADS);
                glVertex2f(x1, y1);
                glVertex2f(x2, y1);
                glVertex2f(x2, y2);
                glVertex2f(x1, y2);
                glEnd();
            }
            if (start_new_rect)
            {
                // Remember the start x,y

                start_x = x - ori_x;
                current_y = y - ori_y;
            }

            if (end_render)
                return false;

            // Remember the ending x of the last rect-included glyph
            end_x = x - ori_x + w;

            return true;
        },
        align, maxWidth);
}

int WhichGlyphAt(const Font *pFont, const char *pUTF8,
                  const float px, const float py,
                  const int align, float maxWidth)
{
    /*
        Important! Index positions must be set to a negative number at start.
        Negative is interpreted as 'unset'.
     */
    int pos = -1,
        leftmost_pos = -1,
        rightmost_pos = -1;

    ThroughText(pFont, pUTF8,
        [&] (const Glyph *pGlyph, const float x, const float y, const int string_pos)
        {
            // Calculate glyph bounding box:
            float ori_x,
                  ori_y,
                  h_adv;

            GetGlyphOrigin(pFont, pGlyph, ori_x, ori_y);

            h_adv = GetHAdv(pFont, pGlyph);

            float x1 = x - ori_x,
                  x2 = x1 + h_adv,
                  y1 = y - ori_y,
                  y2 = y1 + GetLineSpacing(pFont);

            // See if px,py hits the bounding box:

            if (py < y1)
            {
                // point is above this line, we will not encounter it anymore in the upcoming lines

                return false;
            }
            else if (py > y1 && py < y2)
            {
                // point is on this line

                if (px >= x1 && px < x2) // point is inside the glyph's box
                {
                    pos = string_pos;

                    return true; // we might want the next glyph, after kerning
                }
                else if (px < x1 && leftmost_pos < 0) // leftmost_pos not yet set and point is to the left of the glyph
                {
                    leftmost_pos = string_pos;

                    return true;
                }
                else if (px > x2 && pos < 0) // pos is not yet set and point is to the right of the glyph
                {
                    rightmost_pos = string_pos;

                    return true;
                }
            }

            return(pos < 0); // as long as index is -1, keep checking the next glyph
        },
        align, maxWidth);

    if (pos < 0) // no exact match, but maybe the point is on the same line
    {
        if (leftmost_pos >= 0)

            return leftmost_pos;

        else if (rightmost_pos >= 0)

            return rightmost_pos;
    }

    return pos;
}
void CoordsOfGlyph(const Font *pFont, const char *pUTF8, const int pos,
                         float &outX, float &outY,
                         const int align, float maxWidth)
{
    ThroughText(pFont, pUTF8,
        [&] (const Glyph *pGlyph, const float x, const float y, const int string_pos)
        {
            if (pos == string_pos) // is this the glyph we were looking for?
            {
                float ori_x,
                      ori_y;
                GetGlyphOrigin(pFont, pGlyph, ori_x, ori_y);

                // Return its position
                outX = x - ori_x;
                outY = y - ori_y;

                return false;
            }
            return true;
        },
        align, maxWidth);
}
void DimensionsOfText(const Font *pFont, const char *pUTF8,
                       float &outX1, float &outY1, float &outX2, float &outY2,
                       const int align, float maxWidth)
{
    // If the font has no glyphs, no text can exist:
    if (pFont->glyphs.size() <= 0)
    {
        outX1 = outY1 = outX2 = outY2 = 0.0;
        return;
    }

    /*
        At start, set the bounding box minima to unrealistically high numbers
        and the maxima to unrealistically negative numbers. That way, we
        know for sure that they'll be overruled by the glyphs in the text.
     */
    float minX, maxX, minY, maxY;
    minX = minY = 1.0e+15f;
    maxX = maxY = -1.0e+15f;

    ThroughText(pFont, pUTF8,
        [&] (const Glyph *pGlyph, const float x, const float y, const int string_pos)
        {
            // Calculate glyph bounding box:
            float ori_x, ori_y, h_adv;
            GetGlyphOrigin(pFont, pGlyph, ori_x, ori_y);

            h_adv = GetHAdv(pFont, pGlyph);

            float x1 = x - ori_x, x2 = x1 + h_adv,
                  y1 = y - ori_y, y2 = y1 + GetLineSpacing(pFont);

            // Expand text bounding box with glyph's bounding box:
            minX = std::min(x1, minX);
            minY = std::min(y1, minY);
            maxX = std::max(x2, maxX);
            maxY = std::max(y2, maxY);

            return true;
        },
        align, maxWidth);

    outX1 = minX;
    outX2 = maxX;
    outY1 = minY;
    outY2 = maxY;
}
