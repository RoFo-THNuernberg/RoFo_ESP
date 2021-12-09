#pragma once

#include <cstring>
#include <stdint.h>
#include <string>

class SmartBufferPtr
{
    public:
        SmartBufferPtr(uint8_t** buffer_structure, size_t max_ptr, size_t max_ptr_ptr) : 
            _internal_ptr{buffer_structure[0]}, _internal_ptr_ptr{buffer_structure}, _max_ptr{max_ptr}, _max_ptr_ptr{max_ptr_ptr} {}
        
        int operator-(SmartBufferPtr const& bfr) const;

        void operator+=(int incr);

        uint8_t operator[](int incr) const;

        
    private:
        uint8_t* _internal_ptr;
        uint8_t** _internal_ptr_ptr;
        size_t _max_ptr;
        size_t _max_ptr_ptr;

};

template <typename T> void operator<<(T& value, SmartBufferPtr const& bfr)
{   
    uint8_t* value_ptr = (uint8_t*)&value;

    for(int i = 0; i < sizeof(T); i++)
        value_ptr[i] = bfr[i];
}

void operator<<(std::string& str, SmartBufferPtr const& bfr);