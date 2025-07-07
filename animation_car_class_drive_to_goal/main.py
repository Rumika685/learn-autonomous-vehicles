import matplotlib.pyplot as plt
from car import Car  # 上のコードを car.py に保存している場合

fig, ax = plt.subplots()
car = Car("RumiCar", manual_mode=True, ax=ax)

car.drive_to_goal(goal=3, max_steps=10)
plt.show()