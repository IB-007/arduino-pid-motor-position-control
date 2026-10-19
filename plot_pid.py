# python plot_pid.py COM5 -> acts as the Serial Monitor (close the IDE's one), records one move
#                            into data.txt, then plots it.
# python plot_pid.py      -> just plots an existing data.txt.
# Saves data.csv (opens in Excel) and plot.png.
import sys, threading
import matplotlib.pyplot as plt

TS = 0.02  # TS_MS in motor_pid.ino

if len(sys.argv) > 1:
    import serial
    ser = serial.Serial(sys.argv[1], 115200, timeout=1)   # opening the port resets the Uno
    threading.Thread(target=lambda: [ser.write((input() + "\n").encode()) for _ in iter(int, 1)], daemon=True).start()
    with open("data.txt", "w") as f:
        while True:
            line = ser.readline().decode(errors="ignore").strip()
            if not line:
                continue
            f.write(line + "\n")
            if not line.startswith("Set:"):
                print(line)
            if line.startswith("Moving"):
                print("(recording...)")
            if line.startswith(("===================", "Did NOT settle", "Aborted", "RUNAWAY", "STALL")) \
                    and "Settled" not in line:
                break
    ser.close()
rows = [dict(kv.split(":") for kv in l.strip().split(",")) for l in open("data.txt") if l.startswith("Set:")]
t = [i * TS for i in range(len(rows))]
keys = list(rows[0])

with open("data.csv", "w") as f:
    f.write("t," + ",".join(keys) + "\n")
    f.writelines(f"{ti}," + ",".join(r[k] for k in keys) + "\n" for ti, r in zip(t, rows))

fig, (a1, a2) = plt.subplots(2, 1, sharex=True)
for k in ["Set", "Pos"]:
    a1.plot(t, [float(r[k]) for r in rows], label=k)
for k in ["P", "I", "D", "PWM"]:
    a2.plot(t, [float(r[k]) for r in rows], label=k)
a1.set_ylabel("deg"); a2.set_ylabel("PWM"); a2.set_xlabel("time (s)")
a1.legend(); a2.legend(); a1.grid(); a2.grid()
plt.savefig("plot.png", dpi=150)
plt.show()
