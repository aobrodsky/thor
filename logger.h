#ifndef LOGGER_H
#define LOGGER_H

extern int logger_init( char *name, int console );
extern void logger_setname( char *proc );
extern void logger_printf( char *fmt, ... );

#endif
