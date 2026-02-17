#!/bin/bash
cd main
find .. -iname "*.[ch]" -type f -print | ctags -L - -u ../tags
#ctags --c++-kinds=+p --fields=+iaS --extras=+q -a ../tags *.c *.h


