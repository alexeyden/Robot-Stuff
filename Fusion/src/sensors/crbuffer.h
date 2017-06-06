#ifndef CRBUFFER_H
#define CRBUFFER_H

#include <cstdint>
#include <iterator>

// circular buffer
template<typename T, size_t buf_size>
class crbuffer {
public:
    class const_iterator {
    public:
        typedef const_iterator self_type;
        typedef T value_type;
        typedef T& reference;
        typedef T* pointer;
        typedef int difference_type;
        typedef std::forward_iterator_tag iterator_category;

        const_iterator(const crbuffer<T,buf_size>& buf, size_t ptr, bool end) :
            _end(end), _buf(buf) {
            _ptr = ptr;
        }

        self_type operator ++() {
            _ptr++;
            if(_ptr == _buf._eptr && _buf._eptr == buf_size)
                _ptr = 0;

            if(_ptr == _buf._fptr)
                _end = true;
            return *this;
        }

        self_type operator ++(int) {
            self_type prev = *this;
            ++(*this);
            return prev;
        }

        const value_type& operator *() const {
            return _buf._data[_ptr];
        }

        const value_type* operator -> () const {
            return &_buf._data[_ptr];
        }

        bool operator == (const self_type& rhs) const {
            return _buf == rhs._buf && _ptr == rhs._ptr && _end == rhs._end;
        }

        bool operator != (const self_type& rhs) const {
            return _buf != rhs._buf || _ptr != rhs._ptr || _end != rhs._end;
        }

    private:
        bool _end;
        size_t _ptr;
        const crbuffer<T,buf_size> & _buf;
    };

public:
    crbuffer() : _fptr(0), _eptr(0) {
        _data = new T[buf_size];
    }
    ~crbuffer() {
        delete [] _data;
    }

    const_iterator begin() const {
        if(_eptr == buf_size)
            return const_iterator(*this, _fptr, false);
        else
            return const_iterator(*this, 0, false);
    }

    const_iterator end() const {
        return const_iterator(*this, _fptr, true);
    }

    size_t eptr() const {
        return _eptr;
    }

    size_t fptr() const {
        return _fptr;
    }

    const T* data() const {
        return _data;
    }

    void put(const T& item) {
        if(_eptr != buf_size)
            _eptr ++;

        _data[_fptr] = item;
        _fptr ++;

        if(_fptr == buf_size)
            _fptr = 0;
    }

    void reset() {
        _fptr = 0;
        _eptr = 0;
    }

    bool operator ==(const crbuffer<T, buf_size>& rhs) const {
        return _data == rhs._data && _fptr == rhs._fptr && _eptr == rhs._eptr;
    }

    bool operator !=(const crbuffer<T, buf_size>& rhs) const {
        return !((*this) == rhs);
    }

    size_t size() const {
        return buf_size;
    }

    friend class const_iterator;
private:
    size_t _fptr;
    size_t _eptr;

    T* _data;
};

#endif // CRBUFFER_H
