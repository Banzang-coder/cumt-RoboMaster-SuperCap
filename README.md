# cumt-RoboMaster-SuperCap
一种用于Robomaster比赛的双向拓扑超级电容

本设计用于2023年全国赛cubot战队舵轮步兵与摩擦轮英雄，旨在提高规则允许下的闲置功率利用率。

 ![Image text](Screenshots/product.png)
 
# 调试过程遇到的问题(记忆不全，仅记录记得的)
1.在双向调试过程中，存在着有些时候上电就炸板，后来了解到是大多数人做这个拓扑都会犯的错误，是因为buck的缓启动会被反向认作很高的boost导致（已解决）

2.在一次传统模态切换方式调试成功后，我去学长的电赛备赛地点走了一圈，嫖了4个12块钱一个的NMOS，回来后发现模态怎么都无法平滑切换。后来排除了一天发现是MOS问题。（已解决）

3.后来经学长提醒发现使用占空比来进行平滑切换，有效且稳定，比依靠电压切换更加简单。（狠狠批判，临近比赛学长才告诉我能这么搞doge）


# quote
本设计借鉴了很多深圳大学与河北工业大学两位学长的设计。
