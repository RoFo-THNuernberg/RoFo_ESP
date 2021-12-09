#include "SmartBufferPtr.h"

int SmartBufferPtr::operator-(SmartBufferPtr const& bfr) const {
    return (_internal_ptr_ptr - bfr._internal_ptr_ptr) * _max_ptr + _internal_ptr - bfr._internal_ptr;
}

void SmartBufferPtr::operator+=(int incr)
{
    if(_internal_ptr + incr < _internal_ptr_ptr[0] +_max_ptr)
        _internal_ptr += incr;
    else
        _internal_ptr = _internal_ptr_ptr[1] +  (_internal_ptr_ptr[0] + _max_ptr - _internal_ptr + incr);
}

uint8_t SmartBufferPtr::operator[](int incr) const
{   
    uint8_t* elem_ptr = 0;

    if(_internal_ptr + incr < _internal_ptr_ptr[0] +_max_ptr)
        elem_ptr = _internal_ptr + incr;
    else
        elem_ptr = _internal_ptr_ptr[1] +  (_internal_ptr_ptr[0] + _max_ptr - _internal_ptr + incr);

    return elem_ptr[0];
}

void operator<<(std::string& str, SmartBufferPtr const& bfr)
{
    int i = 0;
    
    while(bfr[i] != '\0')
    {
        str.push_back(bfr[i]);
        i++;
    }
}