/*
MIT License

Copyright (c) 2024 Charles Lohr "CNLohr"

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <stdio.h>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#define IMAGEOPL 32

int main( int argc, char ** argv )
{
	if( argc != 2 )
	{
		fprintf( stderr, "Error: Need input image file\n" );
		return -9;
	}

	int w, h, n;
	unsigned char *data = stbi_load( argv[1], &w, &h, &n, 1 );
	if( !data )
	{
		fprintf( stderr, "Error: Failed to load image \"%s\"\n", argv[1] );
	}
	int i;
	printf( "#include <stdint.h>\n" );
	printf( "\n" );
	printf( "#define IMAGEOH  %d\n", h );
	printf( "#define IMAGEOPL %d\n", IMAGEOPL );
	printf( "uint16_t imageout[IMAGEOH*IMAGEOPL] = {\n" );
	int x, y;
	int o;
	for( y = 0; y < h; y++ )
	{
		o = 0;
		for( x = 0; x < w; x++ )
		{
			uint8_t c = data[(x+y*w)];
			if( c > 128 )
			{
				if( o < IMAGEOPL )
				{
					printf( "%d, ", x ); o++;
				}
				else
				{
					fprintf( stderr, "Warning: overload on line %d\n", y );
				}
			}
		}
		for( ; o < IMAGEOPL; o++ )
		{
			printf( "0, " );
		}
		printf( "\n" );
	}
	printf( "};\n" );
}


