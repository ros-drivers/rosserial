#include "flash_converter.h"

FlashConverter go_converter;
  
FlashConverter::FlashConverter()
{
  po_buffer_index = 0;
  pa_buffer[0] = 0;  //zero termination at beginning
  pa_buffer[ FLASH_CONVERTER_BUFFER - 1 ] = 0; // assure zero termination
};
  
const char * FlashConverter::convertFlashStringToConstCharObj( const __FlashStringHelper * ap_in )
{    
  const prog_char *lp_p = (const prog_char *) ap_in ;
  
  int lo_offset = po_buffer_index;
  
  while ( po_buffer_index < FLASH_CONVERTER_BUFFER - 1 ) 
  {
    pa_buffer[ po_buffer_index ] = pgm_read_byte( lp_p++ );
	    
    if ( pa_buffer[ po_buffer_index ] == 0)
    {
      po_buffer_index++;
      
      break;
    }
    
    po_buffer_index++;
  }
  
  return pa_buffer + lo_offset ;
}

void FlashConverter::clearBufferIfAnyObj()
{
  pa_buffer[0] = 0;  //zero termination at beginning
  po_buffer_index = 0;
}

bool FlashConverter::hasOverflowObj()
{
  return ( po_buffer_index >= FLASH_CONVERTER_BUFFER - 1 );
}


const char * FlashConverter::convertToConstChar( const char*  ap_in )
{
  return ap_in;
}

const char * FlashConverter::convertToConstChar( const __FlashStringHelper * ap_in )
{
  return go_converter.convertFlashStringToConstCharObj( ap_in );
}

void FlashConverter::clearBufferIfAny()
{
  go_converter.clearBufferIfAnyObj();
}

bool FlashConverter::hasOverflow()
{
  return go_converter.hasOverflowObj();
}