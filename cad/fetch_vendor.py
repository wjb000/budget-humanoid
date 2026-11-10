#!/usr/bin/env python3
"""Download vendor CAD that is NOT redistributed in this repo (license unclear or too large) into cad/vendor_fetched/.
Only needed to render the real Pi 5 mesh in the 3D model; everything else is already in cad/vendor/.
Each file stays under its vendor's terms: read them before using the STEP files in a product.
  python cad/fetch_vendor.py"""
import io, os, sys, urllib.request, zipfile

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'vendor_fetched')
os.makedirs(OUT, exist_ok=True)

PI5 = 'https://pip-assets.raspberrypi.com/categories/892-raspberry-pi-5/documents/RP-010083-CA-1-rpi-5%203D%20STEP%20-%20No%20Graphics%20small%20file.zip'
CAM3 = 'https://datasheets.raspberrypi.com/camera/camera-module-3-step.zip'
XIAO = 'https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/res/seeed-studio-xiao-esp32s3-3d_model.zip'
POLOLU = 'https://www.pololu.com/file/0J1437/d24v50f5-step-down-voltage-regulator.step'

def get(url):
    print('GET', url)
    req = urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0 budget-humanoid fetch_vendor'})
    return urllib.request.urlopen(req, timeout=120).read()

def unzip(data, want, rename=None):
    z = zipfile.ZipFile(io.BytesIO(data))
    for n in z.namelist():
        if n.startswith('__MACOSX'):
            continue
        base = os.path.basename(n)
        if want(base):
            dst = os.path.join(OUT, (rename or {}).get(base, base))
            open(dst, 'wb').write(z.read(n)); print('  ->', dst)

if __name__ == '__main__':
    unzip(get(PI5), lambda b: b.lower().endswith('.step') or b.upper().startswith('LICENSE'),
          {'rpi-5b_no_graphics.step': 'rpi5_no_graphics.step', 'LICENSE.txt': 'LICENSE-RaspberryPi5.txt'})
    if '--all' in sys.argv:   # optional extras: only dimensions of these are used (as envelopes) by the model
        unzip(get(CAM3), lambda b: b == 'Camera_module_3_std_model_simple.stp')
        unzip(get(XIAO), lambda b: b.lower().endswith('.step'))
        open(os.path.join(OUT, 'pololu_d24v50f5.step'), 'wb').write(get(POLOLU))
    print('done. (cad/vendor_fetched/ is git-ignored)')
