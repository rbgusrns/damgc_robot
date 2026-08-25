#ifndef __ROM_H__
#define __ROM_H__

#include <stdint.h>

void rom_load_all(void);

void maxmin_write_rom(void);
void maxmin_read_rom(void);
void turnvel_write_rom(void);
void turnvel_read_rom(void);
void turnmark_info_write_rom(void);
void turnmark_info_read_rom(void);
void acc_info_write_rom(void);
void acc_info_read_rom(void);
void handle_write_rom(void);
void handle_read_rom(void);
void mark_write_rom(void);
void mark_read_rom(void);
void fast_infor_write_rom(void);
void fast_infor_read_rom(void);

#endif /* __ROM_H__ */
