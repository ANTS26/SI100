# Homework 0: Logisim Evolution
[toc]

## 1. Warming Up

### Have a try: Building a Simple OR Gate

在本次练习中，我们将创建一个 `OR` 门电路，帮助熟悉 Logisim 的基本操作。以下是具体步骤：

1. 单击左侧工具栏中的 `OR` 按钮，将 `OR` 门放置在主窗口的适当位置。
2. 使用 `Input Pin` 按钮在 `OR` 门左侧放置两个输入引脚。
3. 使用 `Output Pin` 按钮在 `OR` 门右侧放置一个输出引脚。
4. 使用 `Select` 工具连接输入引脚到 `OR` 门，并连接 `OR` 门的输出到输出引脚。
5. 使用 `Label` 工具为每个引脚命名，方便识别。
6. 使用 `Poke` 工具更改输入引脚状态，观察 `OR` 门的输出是否符合预期。


---

## 2. Introduction to Logisim's Components
在这部分中，我们介绍一些常用的器件：

### 2.1. Pin
针脚是电路的输出还是输入，取决于它的输出值属性（即 Selection 中的 `Output?` ）。

圆形或圆角矩形表示输出针脚，正方形或矩形表示输入针脚。

输入或输出值都会在组件上显示(打印视图除外)。

> 尝试使用 Pin 进行输入输出。

### 2.2. Tunnel
标签通道的作用类似于导线，但与导线不同的是，连接不是明确绘制的。当你需要连接电路中相隔很远的点时，用标签通道来替代就很有用。下面的插图说明了它是如何工作的：这三个隧道都有相同的标签 a，这三个隧道相当于连接的点。(如果其中一条隧道被标记为其他东西，比如b，那么它将是另一组隧道的一部分)

其主要参数是 “Label（标签）” ，这是一个特别重要的属性，如果两个标签通道的标签名称一样，那么它们相当于之间有导线连接，是连通的。

![](https://picturebed-1306857708.cos.ap-shanghai.myqcloud.com/General_Images/Tunnel.png)
> 尝试使用 Tunnel 代替直接连线，进行电路的构建。

### 2.3. Constant
Constant 输出在 Value（值）属性中指定的值。它只有一个引脚，输出对应位宽属性的值。

其属性包括：
- Data Bits（数据位宽）：指定输出数据的位宽，一个 $n$ 位的常量有 $2^n$ 个可能的值（范围为 $[0,2^{n-1}]$）
- Value（值）：指定常量的值，注意 `0x` 表示的是十六进制

### 2.4. Splitter
分线器可以把一个多位的输入拆分为若干位，也可反过来把若干个若干位的输入合并为一个输出，可设置具体拆分方式。
其参数包括：
- Facing（朝向）：控制元件的朝向
- Fan Out（分流端口数）：设置输出端口的数量
- Bit Width In（输入位宽）：设置输入位宽
- Bit X （第X位）：设置第X位数据在哪个口输出

![](https://picturebed-1306857708.cos.ap-shanghai.myqcloud.com/General_Images/Splitter.png)

### 2.5. 构建子电路
正如 Python 程序可以包含函数一样， Schematic 也可以包含子电路。在这部分实验中，我们将创建几一些子电路来演示它们的用途。

> **重要提示**：Logisim Evolution 规定，不能用关键字（如 `NAND` ）命名子电路，也不能在名称的第一个字符使用数字。



## About other Logisim Guide
你可以参考如下教程进行 Logisim 的学习：
- [CSDN: Logisim 教程](https://blog.csdn.net/Hi_KER/article/details/120928866)

## 3. Exercise

### 作业内容

1. 打开 HW 0 的 Schematic  ( `File -> Open -> ex1.circ` )。
2. 双击左侧菜单电路选择器中的 `NAND1` 打开 `NAND1` 空子电路。（注意末尾的 `1` ；因为有一个名为 `NAND` 的组件，所以我们不能将其称为 `NAND` ）。
3. 在新的 Schematic 窗口中，创建一个简单的 `NAND` 电路，左侧为 2 个输入引脚，右侧为输出引脚。**请不要使用 Gates 文件夹中的内置 `NAND` 门**（即只使用选择工具图标旁边提供的 `AND` 、 `OR` 和 `NOT` 门）。您可以使用选择工具选择输入/输出，并更改窗口左下角的属性 `Label` 来更改输入和输出的标签。
4. 重复上述步骤，再创建几个子电路：
   - `NOR`
   - `XOR`
   - `2-to-1 MUX`
   - `4-to-1 MUX` 
5. 注意事项
   - 请不要更改子电路的名称或创建新的子电路
   - 请在分别命名的电路中工作，否则自动跟踪器将无法正常工作。
   - 请勿使用除 `AND` 、 `OR` 和 `NOT` 以外的任何内置门电路。
   - 建立子电路后，您可以（而且我们鼓励您）用它来建立其他电路。您可以像放置其他元件一样，点击并放置您创建的子电路。
   - 在这次 homework 中，请按照以下规则构建您的电路：
   - `2-to-1 MUX` ：
     -  `Sel`: 0, -> Select `A`
     -  `Sel`: 1, -> Select `B`
   - `4-to-1 MUX` ：
     -  `Sel1`: 0, `Sel2`: 0 -> Select `A`
     -  `Sel1`: 1, `Sel2`: 0 -> Select `B`
     -  `Sel1`: 0, `Sel2`: 1 -> Select `C`
     -  `Sel1`: 1, `Sel2`: 1 -> Select `D`

### 提交

请直接提交1个单文件 `ex1.circ`至Gradescope