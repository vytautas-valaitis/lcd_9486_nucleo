from functools import reduce
from PIL import Image
img = Image.open('i.png')

data = img.tobytes('hex', 'rgb')

im = []

for i in range (64):
  for j in range (64):
    r, g, b, a = img.getpixel((i, j))
    im.append((r, g, b))

  with open("img.i", "wb") as binary_file:
    for i in range(len(im)):
      r = im[i][0]
      b = im[i][1]
      g = im[i][2]
      hb = (((r & 0xf8) & 0xff) | ((g >> 5) & 0xff))
      lb = (((g << 5) & 0xff) | ((b >> 3) & 0xff))
      binary_file.write(hb.to_bytes(1, 'big'))
      binary_file.write(lb.to_bytes(1, 'big'))

