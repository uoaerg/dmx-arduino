syntax on
set number
set tabstop=4
set autoindent
color koehler
set noeb
set vb
set t_vb=

highlight OverLength ctermbg=red ctermfg=white guibg=#592929
match OverLength /\%81v.\+/

au BufNewFile,BufRead *.frag,*.vert,*.vsh,*.fsh set syntax=glsl 
au BufNewFile,BufRead *.md,*.markdown set syntax=markdown
au BufNewFile,BufRead *.numbers,*.diet set syntax=markdown
au BufNewFile,BufRead *.ino set filetype=cpp

set nocompatible
filetype plugin on

execute pathogen#infect() 
