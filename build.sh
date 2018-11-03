#!/usr/bin/env bash

set -e

mkdir ftc_app
git clone --depth=1 https://github.com/ftctechnh/ftc_app.git ftc_app
rm -rf ftc_app/.git

rsync -a 2018/* ftc_app
rm -rf ftc-terrabats