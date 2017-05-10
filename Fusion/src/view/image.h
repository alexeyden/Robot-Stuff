#ifndef IMAGE_H
#define IMAGE_H

#include <cstdint>
#include <cstddef>

#include <fstream>
#include <png.h>

template<typename T = uint8_t>
class image
{
public:
    static image<T>* load(const char* file)
    {
        png_structp png_ptr;
        png_infop info_ptr;

        uint32_t w, h;

        std::ifstream ifile( file, std::ios::binary );

        if( !ifile.is_open() )
            return nullptr;

        char assinatura[8];
        ifile.read( &assinatura[0], 8 * sizeof(char) );

        png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, 0, 0, 0);
        info_ptr = png_create_info_struct( png_ptr );

        auto readFileCallback = []( png_structp png_ptr, png_bytep out, png_size_t count ) {
            png_voidp io_ptr = png_get_io_ptr( png_ptr );
            if( io_ptr == 0 ) {
                return;
            }
            std::ifstream &ifs = *(std::ifstream*)io_ptr;
            ifs.read( (char*)out, count );
        };

        png_set_read_fn( png_ptr, (void*)&ifile, readFileCallback );

        png_set_sig_bytes( png_ptr, 8 );

        png_read_info( png_ptr, info_ptr );

        int depth, color_type;
        png_get_IHDR( png_ptr, info_ptr, (png_uint_32*)&w, (png_uint_32*)&h,
                      &depth, &color_type, 0, 0, 0 );

        png_size_t cols = png_get_rowbytes(png_ptr, info_ptr);

        png_bytepp row_pp = new png_bytep[h];
        uint8_t* data = new uint8_t[ cols * h];

        for( size_t i = 0; i < h; ++i )
        {
            row_pp[i] = (png_bytep)&data[ i * cols ];
        }

        png_read_image( png_ptr, row_pp );
        png_read_end( png_ptr, info_ptr );

        png_destroy_read_struct( &png_ptr, &info_ptr, 0 );

        delete[] row_pp;

        return new image(data, w, h, color_type == 2 ? 3 : 4);
    }

    image(T* d, size_t w, size_t h, size_t ch, bool own = false) :
        _data(d), _width(w), _heigh(h), _channels(ch), _own(own)
    {
    }

    ~image()
    {
        if(_own)
            delete[] _data;
    }

public:
    T* data() { return _data; }
    T* data() const { return _data; }
    size_t width() const { return _width; }
    size_t height() const { return _heigh; }
    size_t channels() const { return _channels; }

private:

    T* _data;
    size_t _width;
    size_t _heigh;
    size_t _channels;
    bool _own;
};

#endif // IMAGE_H
