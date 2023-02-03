#!/bin/bash
# I'd love to add comments line by line but bash won line by line but bash wont let me

tmux \
new-session \
'echo "Hello World" ; bash'\; \
split-window \
'echo "Hello World" ; bash'\; \
split-window -h \
'echo "Hello World" ; bash'\; \
split-window \
'echo "Hello World" ; bash'\; \
select-layout even-horizontal\; \
select-pane -t 0 \; \
split-window  -v \
'echo "Hello World" ; bash'\; \
select-pane -t 2 \; \
split-window  -v \
'echo "Hello World" ; bash'\; \
select-pane -t 4 \; \
split-window  -v \
'echo "Hello World" ; bash'\; \
select-pane -t 6 \; \
split-window  -v \
'echo "Hello World" ; bash'\; \