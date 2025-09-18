from PIL import Image

# Open your PNG
img = Image.open("fzj.png")

# Save as .ico (you can specify multiple sizes for Windows icons)
img.save("fzj.ico", format="ICO", sizes=[(16,16), (32,32), (48,48), (64,64), (128,128), (256,256)])