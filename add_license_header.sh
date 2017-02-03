#!/bin/bash

shopt -s globstar nullglob extglob
for i in **/*.@(h|cpp) # or whatever other pattern...
do
  if ! grep -q Copyright $i
  then
    cat LICENSE.txt $i >$i.new && mv $i.new $i
  fi
done

