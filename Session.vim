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
badd +40 ~/ros1_ws/src/rviz-interactive-markers/src/int_marker.cpp
badd +50 ~/ros1_ws/src/rviz-interactive-markers/include/rviz-interactive-markers/int_marker.hpp
badd +2 ~/ros1_ws/src/rviz-interactive-markers/config/int_marker.yaml
badd +6 ~/ros1_ws/src/rviz-interactive-markers/launch/start_int_marker.launch
badd +38 ~/ros1_ws/src/rviz-interactive-markers/src/display_int_markers.cpp
badd +15 ~/ros1_ws/src/rviz-interactive-markers/src/display_int_markers_node.cpp
badd +0 ./
badd +1 ~/ros1_ws/src/rviz-interactive-markers/config/display_int_markers_params.yaml
badd +0 fugitive:///home/brina/ros1_ws/src/rviz-interactive-markers/.git//
argglobal
%argdel
$argadd ./
edit ~/ros1_ws/src/rviz-interactive-markers/src/int_marker.cpp
let s:save_splitbelow = &splitbelow
let s:save_splitright = &splitright
set splitbelow splitright
wincmd _ | wincmd |
split
1wincmd k
wincmd w
let &splitbelow = s:save_splitbelow
let &splitright = s:save_splitright
wincmd t
let s:save_winminheight = &winminheight
let s:save_winminwidth = &winminwidth
set winminheight=0
set winheight=1
set winminwidth=0
set winwidth=1
wincmd =
argglobal
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
let s:l = 271 - ((13 * winheight(0) + 12) / 24)
if s:l < 1 | let s:l = 1 | endif
keepjumps exe s:l
normal! zt
keepjumps 271
normal! 0
wincmd w
argglobal
if bufexists(fnamemodify("fugitive:///home/brina/ros1_ws/src/rviz-interactive-markers/.git//", ":p")) | buffer fugitive:///home/brina/ros1_ws/src/rviz-interactive-markers/.git// | else | edit fugitive:///home/brina/ros1_ws/src/rviz-interactive-markers/.git// | endif
if &buftype ==# 'terminal'
  silent file fugitive:///home/brina/ros1_ws/src/rviz-interactive-markers/.git//
endif
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
let s:l = 1 - ((0 * winheight(0) + 12) / 24)
if s:l < 1 | let s:l = 1 | endif
keepjumps exe s:l
normal! zt
keepjumps 1
normal! 0
wincmd w
2wincmd w
wincmd =
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
