let SessionLoad = 1
let s:so_save = &g:so | let s:siso_save = &g:siso | setg so=0 siso=0 | setl so=-1 siso=-1
let v:this_session=expand("<sfile>:p")
silent only
silent tabonly
cd ~/ros1_ws/src/rviz-interactive-markers
if expand('%') == '' && !&modified && line('$') <= 1 && getline(1) == ''
  let s:wipebuf = bufnr('%')
endif
let s:shortmess_save = &shortmess
if &shortmess =~ 'A'
  set shortmess=aoOA
else
  set shortmess=aoO
endif
badd +275 ~/ros1_ws/src/rviz-interactive-markers/src/int_marker.cpp
badd +42 ~/ros1_ws/src/rviz-interactive-markers/include/rviz-interactive-markers/int_marker.hpp
badd +1 ~/ros1_ws/src/rviz-interactive-markers/src/int_marker_node.cpp
badd +2 ~/ros1_ws/src/rviz-interactive-markers/config/simple_marker_params.yaml
badd +2 ~/ros1_ws/src/rviz-interactive-markers/config/int_marker.yaml
badd +6 ~/ros1_ws/src/rviz-interactive-markers/launch/start_int_marker.launch
badd +38 ~/ros1_ws/src/rviz-interactive-markers/src/display_int_markers.cpp
badd +15 ~/ros1_ws/src/rviz-interactive-markers/src/display_int_markers_node.cpp
badd +10 ~/ros1_ws/src/rviz-interactive-markers/README.md
argglobal
%argdel
$argadd ./
edit ~/ros1_ws/src/rviz-interactive-markers/README.md
let s:save_splitbelow = &splitbelow
let s:save_splitright = &splitright
set splitbelow splitright
let &splitbelow = s:save_splitbelow
let &splitright = s:save_splitright
wincmd t
let s:save_winminheight = &winminheight
let s:save_winminwidth = &winminwidth
set winminheight=0
set winheight=1
set winminwidth=0
set winwidth=1
argglobal
balt ~/ros1_ws/src/rviz-interactive-markers/src/int_marker.cpp
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let &fdl = &fdl
let s:l = 10 - ((9 * winheight(0) + 25) / 51)
if s:l < 1 | let s:l = 1 | endif
keepjumps exe s:l
normal! zt
keepjumps 10
normal! 0
tabnext 1
if exists('s:wipebuf') && len(win_findbuf(s:wipebuf)) == 0 && getbufvar(s:wipebuf, '&buftype') isnot# 'terminal'
  silent exe 'bwipe ' . s:wipebuf
endif
unlet! s:wipebuf
set winheight=1 winwidth=20
let &shortmess = s:shortmess_save
let &winminheight = s:save_winminheight
let &winminwidth = s:save_winminwidth
let s:sx = expand("<sfile>:p:r")."x.vim"
if filereadable(s:sx)
  exe "source " . fnameescape(s:sx)
endif
let &g:so = s:so_save | let &g:siso = s:siso_save
let g:this_session = v:this_session
let g:this_obsession = v:this_session
doautoall SessionLoadPost
unlet SessionLoad
" vim: set ft=vim :
