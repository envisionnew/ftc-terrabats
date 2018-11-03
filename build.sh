#!/usr/bin/env bash

set -e

mkdir ftc_app
git clone --depth=1 https://github.com/ftctechnh/ftc_app.git ftc_app
rm -rf ftc_app/.git

rsync -a 2018/* ftc_app
rm -rf ftc-terrabats

sudo mkdir -p "$ANDROID_SDK/licenses"
sudo echo -e "\n8933bad161af4178b1185d1a37fbf41ea5269c55" > "$ANDROID_SDK/licenses/android-sdk-license"
sudo echo -e "\n84831b9409646a918e30573bab4c9c91346d8abd" > "$ANDROID_SDK/licenses/android-sdk-preview-license"