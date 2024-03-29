# Labs results.
## lab1: Rigidbody collision
Impulse method

![rigid_impulse](https://github.com/lyk-ohlyk/games103_labs/assets/18349598/624eee4d-67e8-4d26-be22-48050bfec4e9)

Shape matching

![rigid_shap_matching](https://github.com/lyk-ohlyk/games103_labs/assets/18349598/48ff0f17-a99c-4b07-8605-f90502b2caa2)

## lab2: cloth simulation
implicit_model

迭代次数直接影响了弹簧质点回归到理论值的时间，导致你很难对不同的物体同时控制合理的迭代次数区间。游戏中应该会优先选择其他方法来做。

![cloth3](https://github.com/lyk-ohlyk/games103_labs/assets/18349598/9d1e7d2c-63c3-48b1-9a08-1e37ee371fce)

PBD model

可以看到 PBD 本身不会太依赖迭代次数来保持稳定，迭代次数越多意味着弹簧之间的约束越强（布料约紧致）；虽然不“物理”，但效果还是比隐式方法简单有效很多。

![cloth_PBD](https://github.com/lyk-ohlyk/games103_labs/assets/18349598/7843ab1c-9118-4951-8fcf-59b15cdacf80)
