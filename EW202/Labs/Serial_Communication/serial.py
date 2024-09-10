import ttyacm
tty = ttyacm.open(1)

msg = tty.readline()

print("Start")
print(f"Message: {msg}")
print("End")

tty.print(*"pico to MATLAB")