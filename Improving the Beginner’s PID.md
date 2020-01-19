---
typora-root-url: ./
---

# 改善初学者的PID

[TOC]

## 一、简介

结合新的Arduino PID库的发布，我决定发布此系列文章。 最后一个库虽然可靠，但实际上并没有任何代码说明。 这次围绕该计划详细解释代码为何如此。 我希望这对两类人有用：

- 直接对Arduino PID库中发生的事情感兴趣的人将获得详细说明。

- 任何编写自己的PID算法的人都可以看看我的工作方式，并借用他们喜欢的任何东西。

这将是一个艰难的口号，但我认为我发现了一种不太痛苦的方式来解释我的代码。 我将从所谓的“初学者的PID”开始。然后逐步进行改进，直到获得具有高效，鲁棒性的pid算法。

### 初学者的PID

这是每个人首先学习的PID公式：

![img](.\images\pidalgorithm.png)

这使得几乎每个人都编写以下PID控制器：

![image-20200116194431713](/images/introduction.png)

```c
/*working variables*/
unsigned long lastTime;
double Input, Output, Setpoint;
double errSum, lastErr;
double kp, ki, kd;
void Compute()
{
   /*How long since we last calculated*/
   unsigned long now = millis();
   double timeChange = (double)(now - lastTime);
  
   /*Compute all the working error variables*/
   double error = Setpoint - Input;
   errSum += (error * timeChange);
   double dErr = (error - lastErr) / timeChange;
  
   /*Compute PID Output*/
   Output = kp * error + ki * errSum + kd * dErr;
  
   /*Remember some variables for next time*/
   lastErr = error;
   lastTime = now;
}
  
void SetTunings(double Kp, double Ki, double Kd)
{
   kp = Kp;
   ki = Ki;
   kd = Kd;
}
```

Compute（）被定期或不定期调用，并且效果很好。不过，本系列不是关于“效果很好”的内容。如果要将该代码转换为与工业PID控制器类似的代码，则必须解决一些问题：

1. **采样时间** - 如果定期对PID算法进行评估，则其效果最佳。如果算法知道此间隔，我们还可以简化一些内部数学运算。
2. **微分冲击** - 这不是最大的问题，但是很容易解决，因此我们将这样做。
3. **动态调整参数** - 一种好的PID算法可以在不影响内部工作的情况下更改调整参数。
4. **缓解积分饱和** - 我们将介绍什么是缓解积分饱和，并实施具有附带好处的解决方案。
5. **开/关（自动/手动）**- 在大多数应用中，有时需要关闭PID控制器并手动调节输出，而不会干扰控制器。
6. **初始化** - 控制器首次开启时，我们希望进行”无扰动的传输“，也就是说，我们不希望输出突然变为某个新值。
7. **控制方向** - 最后一个不是鲁棒性本身名称的更改，它旨在确保用户输入具有正确符号的调优参数。
8. **新增 测量比例（Proportional on Measurement）** - 添加这个特性使得它更加容易控制特定类型的过程。

解决所有问题后，我们将获得一个可靠的PID算法。我们也会（并非偶然）拥有最新版本的Arduino PID 库中使用的代码。因此，无论你在尝试编写自己的算法，还是试图了解PID库正在发生的事情，希望对你有所帮助。让我们开始吧。

## 二、采样时间

### 问题

初学者的PID被设置为不定期调用。这导致了两个问题：

1. 你无法从PID中获得一致的表现，因为它有时会频繁调用，有时不会。
2. 你需要做额外的数学运算来计算微分和积分，因为他们都取决于时间的变化。　

### 解决方案

确保定期调用PID。我决定执行此操作的方法是指定每个周期调用一次计算函数。根据预定的采样时间，PID计算函数决定是应该计算还是返回。

一旦知道以恒定的间隔对PID进行评估，就可以简化微分和积分计算。

### 代码

![image-20200116194227550](/images/sampletime.png)

```c
/*working variables*/
unsigned long lastTime;
double Input, Output, Setpoint;
double errSum, lastErr;
double kp, ki, kd;
int SampleTime = 1000; //1 sec
void Compute()
{
   unsigned long now = millis();
   int timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
      /*Compute all the working error variables*/
      double error = Setpoint - Input;
      errSum += error;
      double dErr = (error - lastErr);
 
      /*Compute PID Output*/
      Output = kp * error + ki * errSum + kd * dErr;
 
      /*Remember some variables for next time*/
      lastErr = error;
      lastTime = now;
   }
}
 
void SetTunings(double Kp, double Ki, double Kd)
{
  double SampleTimeInSec = ((double)SampleTime)/1000;
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;
}
 
void SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}
```

现在，在第10和11行，该算法自行决定是否需要计算。 另外，由于我们现在知道采样之间的时间是相同的，因此我们不需要不断地乘以时间变化。 我们只需适当地调整Ki和Kd（第31和32行），结果在数学上是等效的，但效率更高。
虽然这样做有点瑕疵。 如果用户决定在操作过程中更改采样时间，则需要重新调整Ki和Kd以反映此新更改。 这就是第39-42行的全部内容。
另请注意，我在第29行将采样时间转换为秒。严格来说，这不是必需的，但是这样允许用户以1 / sec和s的单位输入Ki和Kd，而不是1 / mS和mS。

### 结果

上面的更改为我们做了三件事:

1. 不管调用Compute()的频率如何，PID算法都会以固定的时间间隔进行计算【第11行】
2. 由于时间相减【第10行】当millis()重新变为0时将不会有问题。那只会每55天发生一次，但我们需要防止吗？
3. 我们不需要再乘除时间变化。由于他是一个常数，因此我们可以将其从计算代码中移出【第15、16行】，然后将其于调整常数一起输入【第31、32行】。虽然从数学意义上讲，他们的计算结果相同，但是这样做省去了每次PID计算中的一次乘法和一次除法运算。

### 关于中断的旁注

如果此PID应用在微控制器上，则可以作为使用中断的一个很好的依据。SetSampleTime是位置为中断频率，时间到时Compute()将被调用。在这种情况下，则不需要9-12、23、24行。如果你的PID实现计划这么做，那就去吧！不过请继续阅读本系列的文章。你一定会从后续的修改中受益。

我没有使用中断有三点原因：

1. 就本系列的文章而言，并不是每个人都能够使用中断。
2. 如果你想同时实现多个PID控制器，事情将会变得棘手。
3. 老实说，这不是我想做的。吉米·罗杰斯（Jimmie Rodgers）在为我校对该系列作品时提出了建议。我可能决定在PID库的将来版本中使用中断。

## 三、微分冲击

### 问题

这次修改将会微调微分项，目的是消除被称为“微分冲击”的现象。

![img](/images/DonE.png)

上图显示了这个问题。由于error = SetPoint - Input，对Setpoint的任何更改都会导致error瞬间改变。这种变化的导数是无穷大（实际上，由于dt不为0，所以它实际上是一个非常大的数字）该数字被带入PID的方程，这会导致输出出现不希望的峰值。幸运的是，有一种简单的方法可以摆脱这种情况。

### 解决方案

![img](/images/DonMExplain.png)

结果表明，除设定值(Setpoint)值更改的情况外，偏差(Error)的导数等于输入(Input)的负导数。这将是一个完美的解决方法。我们减去（Kd * Input的导数），而不是添加（Kd * Error的导数）。这被称为使用”测量导数(Derivative on Measurement)“。

### 代码

![image-20200117105416416](/images/DerivativeKick.png)

```c
/*working variables*/
unsigned long lastTime;
double Input, Output, Setpoint;
double errSum, lastInput;
double kp, ki, kd;
int SampleTime = 1000; //1 sec
void Compute()
{
   unsigned long now = millis();
   int timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
      /*Compute all the working error variables*/
      double error = Setpoint - Input;
      errSum += error;
      double dInput = (Input - lastInput);
 
      /*Compute PID Output*/
      Output = kp * error + ki * errSum - kd * dInput;
 
      /*Remember some variables for next time*/
      lastInput = Input;
      lastTime = now;
   }
}
 
void SetTunings(double Kp, double Ki, double Kd)
{
  double SampleTimeInSec = ((double)SampleTime)/1000;
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;
}
 
void SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}
```

这里的修改相当容易。我们将+dError替代为-dInput。我们现在记录lastInput而非lastError。

### 结果

![img](/images/DonM.png)

这就是哪些修改给我们带来的。注意输入值仍然看以来一样，因此我们得到了相同的性能，但是每次设定值更改时，我们都不会产生巨大的输出峰值。

这可能重要也可能不重要，这完全取决于你的应用程序对输出峰值的敏感程度。在我看来，并不需要做太多的工作就可以避免尖峰，为什么不这样做呢？

## 四、动态调整参数

### 问题

对任何可靠的PID算法而言，必须具备在系统运行时更改调节参数的能力。

![img](/images/BadIntegral.png)

如果你在系统运行时尝试更改调节参数，初学者PID表现的有点疯狂。让我们探究下原因，这是初学者的PID的参数做了上述变化前后的情况。

![img](/images/BadIntegralCode.png)

因此我们可以立刻将问题归咎于积分项。当参数改变时，这是唯一发生剧烈变化的东西。为什么会这样呢？这于初学者对积分的解释有关：

![img](/images/BadIntegralEqn1.png)

在Ki值被更改之前，这种解释都非常有效。然后突然将新的Ki值乘以全部误差的累积之和。这并不是我们想要的。我们只想影响事情往好的方向发展。

### 解决方案

我知道有几种方法可以解决这个问题。在过去的库中我使用的是缩放errSum的方法。将Ki值加倍，将errSum减半。这样可以防止积分项发生突变，并且可以正常工作。这样做有点笨拙，我又想出了一个更优雅的方法（我不可能是第一个想到这个方法的人，但我的确是一个人想到了。）

这个解决方法需要一点基本的代数知识（或者是微积分？）

![img](/images/GoodIntegralEqn.png)

我们没有将Ki置于积分之外，而是将其放入了积分之内。看起来似乎我们什么都没做，但我们会发现实际上这有很大的不同。

现在，我们采用误差并将其乘以当时的Ki。 然后，我们存储哪些的总和。 当Ki发生变化时，就不会有突变了，因为可以说所有旧Ki都已经在“银行”中了。 我们无需额外的数学运算即可平滑传递。 它可能会让我成为一个极客，但我认为这很性感。

### 代码

![image-20200117120453117](/images/TuringChanges.png)

```c
/*working variables*/
unsigned long lastTime;
double Input, Output, Setpoint;
double ITerm, lastInput;
double kp, ki, kd;
int SampleTime = 1000; //1 sec
void Compute()
{
   unsigned long now = millis();
   int timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
      /*Compute all the working error variables*/
      double error = Setpoint - Input;
      ITerm += (ki * error);
      double dInput = (Input - lastInput);
 
      /*Compute PID Output*/
      Output = kp * error + ITerm - kd * dInput;
 
      /*Remember some variables for next time*/
      lastInput = Input;
      lastTime = now;
   }
}
 
void SetTunings(double Kp, double Ki, double Kd)
{
  double SampleTimeInSec = ((double)SampleTime)/1000;
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;
}
 
void SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}
```

因此我们用复合变量【第4行】替换了errSum变量。它计算了Ki * error的和，而不仅仅是误差【第15行】。另外，由于Ki位于ITerm中，因此已经从主PID计算中删除了【第19行】。

### 结果

![img](/images/GoodIntegral.png)

![img](/images/GoodIntegralCode.png)

那么这是如何解决问题的呢。 在更改ki之前，它会重新调整整个误差的总和。 使用此代码，先前的误差仍然保持不变。

## 五、积分饱和

### 问题

![img](/images/Windup.png)

积分饱和是一个陷阱，他可能比任何其他的内容对初学者有更多的要求。当PID认为自己可以做一些自己做不到的事情时就会发生这种情况。例如，Arduino上的PWM输出接受0-255之间的值。默认情况下，PID不知道这一点。如果它认为300-400-500可以使用，它将尝试使用那些期望得到所需的值。由于实际上该值固定在255，因此它将继续尝试越来越大的数值而没有任何作用。

问题以怪异的滞后形式显现出来。上方我们可以看到输出超出外部极限。当设定值减小后，输出值必须先减小然后才能降到255线以下。

### 解决方案---步骤1（积分限幅）

![img](/images/No-Windup.png)

这有几种方法可以缓解积分饱和，但是我选择的方法如下：告诉PID的输出极限是什么。下面的代码中可以看到有一个SetOuputLimits()函数。一旦达到了任一限制，PID便停止求和（积分）。他知道无事可做。由于输出不会饱和，因此当设定值下降到可以执行某项操作的范围时，我们会立刻得到响应。

### 解决方案---步骤2（输出限幅）

不过请注意，在上图中，尽管我们消除了积分饱和滞后，但并没有完全解决。 pid认为发送的内容与发送的内容之间仍然存在差异。为什么？比例项和（在较小程度上）微分项。
即使已对积分项进行了安全钳位，P和D仍会添加上他们两的部分，使其结果高于输出限制。我认为这是不可接受的。如果用户调用了一个名为“ SetOutputLimits”的函数，则他们必须假设这意味着“输出将保持在这些值之内。”因此，对于第2步，我们做出一个有效的假设。除了限制积分项，我们还限制输出值，使其保持在我们期望的位置。
（注意：您可能会问为什么我们需要同时钳制两个。如果我们无论如何都要进行输出，为什么还要分别钳制积分呢？如果我们所做的只是钳制输出，那么积分术语将追溯到越来越多。尽管在升压过程中输出看起来不错，但在降压过程中我们会看到明显的滞后。）

### 代码

![image-20200117161530482](/images/ResetWindup1.png)

![image-20200117161628845](/images/ResetWindup2.png)

```c
/*working variables*/
unsigned long lastTime;
double Input, Output, Setpoint;
double ITerm, lastInput;
double kp, ki, kd;
int SampleTime = 1000; //1 sec
double outMin, outMax;
void Compute()
{
   unsigned long now = millis();
   int timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
      /*Compute all the working error variables*/
      double error = Setpoint - Input;
      ITerm+= (ki * error);
      if(ITerm> outMax) ITerm= outMax;
      else if(ITerm< outMin) ITerm= outMin;
      double dInput = (Input - lastInput);
 
      /*Compute PID Output*/
      Output = kp * error + ITerm- kd * dInput;
      if(Output > outMax) Output = outMax;
      else if(Output < outMin) Output = outMin;
 
      /*Remember some variables for next time*/
      lastInput = Input;
      lastTime = now;
   }
}
 
void SetTunings(double Kp, double Ki, double Kd)
{
  double SampleTimeInSec = ((double)SampleTime)/1000;
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;
}
 
void SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}
 
void SetOutputLimits(double Min, double Max)
{
   if(Min > Max) return;
   outMin = Min;
   outMax = Max;
    
   if(Output > outMax) Output = outMax;
   else if(Output < outMin) Output = outMin;
 
   if(ITerm> outMax) ITerm= outMax;
   else if(ITerm< outMin) ITerm= outMin;
}
```

一个新的允许用户可以指定输出限制的函数被添加【52-63行】。这些限制用来钳制积分项（I-Term）和输出（Output）。

### 结果

![img](/images/No-Winup-Clamped.png)

正如我们看到的，积分饱和被消除了。另外，输出保持在我们想要的位置。这意味着没有必要对输出进行额外的钳制。如果你想输出的变化范围是23到167，你可以将其设置为输出限制。

## 六、开/关PID控制

### 问题

拥有PID控制器就好了，有时候您根本不在乎它在说什么。

![img](/images/BadForcedOutput.png)

假设您要在程序中的某个时刻将输出强制为某个值（例如0），当然可以在调用例程中执行此操作：

```c
void loop()
{
Compute();
Output=0;
}
```

这样无论PID的怎么说，都可以覆盖他的值。但是，这在实践中是一个可怕的想法。PID会变得非常疑惑：“我一直在改变输出值，什么也没发生，出了什么事情？！让我改变一个更大的输出值。”结果，当你停止覆写输出并切换回PID时，输出值可能会立刻发生巨大的改变。

### 解决方案

解决此问题的方法是有一种方法可以打开和关闭PID。 这些状态的常用术语是“手动”（我将手动调节值）和“自动”（PID将自动调节输出）。 让我们看看如何通过代码完成此操作：

![image-20200117164826151](/images/onoff1.png)

![image-20200117164939703](/images/onoff2.png)

```c
/*working variables*/
unsigned long lastTime;
double Input, Output, Setpoint;
double ITerm, lastInput;
double kp, ki, kd;
int SampleTime = 1000; //1 sec
double outMin, outMax;
bool inAuto = false;
 
#define MANUAL 0
#define AUTOMATIC 1
 
void Compute()
{
   if(!inAuto) return;
   unsigned long now = millis();
   int timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
      /*Compute all the working error variables*/
      double error = Setpoint - Input;
      ITerm+= (ki * error);
      if(ITerm> outMax) ITerm= outMax;
      else if(ITerm< outMin) ITerm= outMin;
      double dInput = (Input - lastInput);
 
      /*Compute PID Output*/
      Output = kp * error + ITerm- kd * dInput;
      if(Output > outMax) Output = outMax;
      else if(Output < outMin) Output = outMin;
 
      /*Remember some variables for next time*/
      lastInput = Input;
      lastTime = now;
   }
}
 
void SetTunings(double Kp, double Ki, double Kd)
{
  double SampleTimeInSec = ((double)SampleTime)/1000;
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;
}
 
void SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}
 
void SetOutputLimits(double Min, double Max)
{
   if(Min > Max) return;
   outMin = Min;
   outMax = Max;
    
   if(Output > outMax) Output = outMax;
   else if(Output < outMin) Output = outMin;
 
   if(ITerm> outMax) ITerm= outMax;
   else if(ITerm< outMin) ITerm= outMin;
}
 
void SetMode(int Mode)
{
  inAuto = (Mode == AUTOMATIC);
}
```

一个相当简单的解决方案。 如果您未处于自动模式，请立即退出计算函数，而无需调整输出或任何内部变量。

### 结果

![img](/images/BetterForcedOutput.png)

的确，您可以通过不从调用例程中调用Compute来达到类似的效果，但是这种解决方案可以保留PID的工作原理，这正是我们所需要的。 通过将内容保持在内部，我们可以跟踪所处的模式，更重要的是，它使我们知道何时更改模式。 这就引出了下一个问题……

## 七、初始化

### 问题

在上一节中，我们实现了打开和关闭PID的功能。 我们关闭了它，但是现在让我们看一下重新打开它会发生什么：

![img](/images/NoInitialization.png)

呀！PID跳回到它发送的最后一个输出值，然后从那里开始调整。 这会导致我们不愿遇到的输入颠簸。

### 解决方案

这个很容易修复，因为我们知道什么时候（从手动到自动），所以我们只需要初始化一些东西就可以实现平稳的转换。这意味着对存储的两个工作变量（ITerm和lastInput）进行处理，以防止输出跳跃。

### 代码

![image-20200117200914625](/images/Initialization1.png)

![image-20200117201013837](/images/Initialization2.png)

```c
/*working variables*/
unsigned long lastTime;
double Input, Output, Setpoint;
double ITerm, lastInput;
double kp, ki, kd;
int SampleTime = 1000; //1 sec
double outMin, outMax;
bool inAuto = false;
 
#define MANUAL 0
#define AUTOMATIC 1
 
void Compute()
{
   if(!inAuto) return;
   unsigned long now = millis();
   int timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
      /*Compute all the working error variables*/
      double error = Setpoint - Input;
      ITerm+= (ki * error);
      if(ITerm> outMax) ITerm= outMax;
      else if(ITerm< outMin) ITerm= outMin;
      double dInput = (Input - lastInput);
 
      /*Compute PID Output*/
      Output = kp * error + ITerm- kd * dInput;
      if(Output> outMax) Output = outMax;
      else if(Output < outMin) Output = outMin;
 
      /*Remember some variables for next time*/
      lastInput = Input;
      lastTime = now;
   }
}
 
void SetTunings(double Kp, double Ki, double Kd)
{
  double SampleTimeInSec = ((double)SampleTime)/1000;
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;
}
 
void SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}
 
void SetOutputLimits(double Min, double Max)
{
   if(Min > Max) return;
   outMin = Min;
   outMax = Max;
    
   if(Output > outMax) Output = outMax;
   else if(Output < outMin) Output = outMin;
 
   if(ITerm> outMax) ITerm= outMax;
   else if(ITerm< outMin) ITerm= outMin;
}
 
void SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto && !inAuto)
    {  /*we just went from manual to auto*/
        Initialize();
    }
    inAuto = newAuto;
}
 
void Initialize()
{
   lastInput = Input;
   ITerm = Output;
   if(ITerm> outMax) ITerm= outMax;
   else if(ITerm< outMin) ITerm= outMin;
}
```

我们修改了SetMode（…）以检测从手动到自动的过渡，并添加了初始化功能。它设置ITerm=Output来处理积分项，设置lastInput=Input来防止导数尖峰。比例项不依赖于过去的任何信息，因此不需要任何初始化。

### 结果

![img](/images/Initialization.png)

从上图中我们可以看到，正确的初始化会导致从手动到自动的无障碍转换：这正是我们所追求的。

### 更新：为什么ITerm = 0？
最近，我收到了很多问题，问为什么初始化时不设置ITerm = 0。 作为回答，我请您考虑以下情形：pid是手动的，并且用户已将输出设置为50。一段时间后，该过程稳定为输入75.2。 用户设定设定值75.2并打开pid。 应该怎么办？
我认为切换到自动后，输出值应保持在50。由于P和D项将为零，因此发生这种情况的唯一方法是将ITerm初始化为Output的值。
如果您需要将输出初始化为零，则无需更改上面的代码。 只需在调用例程中设置Output = 0，然后再将PID从“手动”转换为“自动”即可。

## 八、方向

### 问题

PID将连接的过程分为两类：正作用和反作用。 到目前为止，我展示的所有示例都是直接方式。 即，输出的增加导致输入的增加。 对于反向作用过程，则相反。 例如，在冰箱中，冷却的增加导致温度下降。 为了使初学者PID可以逆向工作，kp，ki和kd的符号都必须为负。
这本身不是问题，但是用户必须选择正确的符号，并确保所有参数都具有相同的符号。

### 解决方案

为了简化这个过程，我要求kp，ki和kd均> = 0。 如果用户连接到反向过程，则他们使用SetControllerDirection函数单独指定该过程。 这样可以确保所有参数都具有相同的符号，并希望使事情更直观。

### 代码

![image-20200117203358347](/images/direction1.png)

![image-20200117203540071](/images/direction2.png)

![image-20200117203743717](/images/direction3.png)

```c
/*working variables*/
unsigned long lastTime;
double Input, Output, Setpoint;
double ITerm, lastInput;
double kp, ki, kd;
int SampleTime = 1000; //1 sec
double outMin, outMax;
bool inAuto = false;
 
#define MANUAL 0
#define AUTOMATIC 1
 
#define DIRECT 0
#define REVERSE 1
int controllerDirection = DIRECT;
 
void Compute()
{
   if(!inAuto) return;
   unsigned long now = millis();
   int timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
      /*Compute all the working error variables*/
      double error = Setpoint - Input;
      ITerm+= (ki * error);
      if(ITerm > outMax) ITerm= outMax;
      else if(ITerm < outMin) ITerm= outMin;
      double dInput = (Input - lastInput);
 
      /*Compute PID Output*/
      Output = kp * error + ITerm- kd * dInput;
      if(Output > outMax) Output = outMax;
      else if(Output < outMin) Output = outMin;
 
      /*Remember some variables for next time*/
      lastInput = Input;
      lastTime = now;
   }
}
 
void SetTunings(double Kp, double Ki, double Kd)
{
   if (Kp<0 || Ki<0|| Kd<0) return;
 
  double SampleTimeInSec = ((double)SampleTime)/1000;
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;
 
  if(controllerDirection ==REVERSE)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
}
 
void SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}
 
void SetOutputLimits(double Min, double Max)
{
   if(Min > Max) return;
   outMin = Min;
   outMax = Max;
 
   if(Output > outMax) Output = outMax;
   else if(Output < outMin) Output = outMin;
 
   if(ITerm > outMax) ITerm= outMax;
   else if(ITerm < outMin) ITerm= outMin;
}
 
void SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto == !inAuto)
    {  /*we just went from manual to auto*/
        Initialize();
    }
    inAuto = newAuto;
}
 
void Initialize()
{
   lastInput = Input;
   ITerm = Output;
   if(ITerm > outMax) ITerm= outMax;
   else if(ITerm < outMin) ITerm= outMin;
}
 
void SetControllerDirection(int Direction)
{
   controllerDirection = Direction;
}
```

### PID完成
到此为止。 我们已经将“初学者的PID”变成了我目前所知道的最强大的控制器。 对于那些正在寻找PID库详细说明的读者，希望您能从中得到帮助。 对于那些编写自己的PID的人，希望您能够收集一些想法，从而节省一些时间。
最后两个注意事项：

1. 如果本系列中的某些内容看起来不对，请通知我。 我可能错过了一些东西，或者可能只是需要在我的解释中更清楚一些。 无论哪种方式，我都想知道。
2. 这只是基本的PID。 为了简化起见，我故意遗漏了许多其他问题。 让我烦恼的是：使用速度而不是位置来进行前馈，重置领带，整数数学，不同的pid形式。 如果有兴趣让我探索这些主题，请告诉我。

## 九、测量比例---代码

在上一篇文章中，我将所有时间都花在解释“比例测量”的好处上。 在这篇文章中，我将解释代码。 人们似乎很欣赏我上次解释事情的逐步方式，所以我将在这里做。 下面的3条详细介绍了我如何将PonM添加到PID库中。

### 第一遍修改–初始输入和比例模式选择

```c
/*working variables*/
unsigned long lastTime;
double Input, Output, Setpoint;
double ITerm, lastInput;
double kp, ki, kd;
int SampleTime = 1000; //1 sec
double outMin, outMax;
bool inAuto = false;
  
#define MANUAL 0
#define AUTOMATIC 1
  
#define DIRECT 0
#define REVERSE 1
int controllerDirection = DIRECT;
  
#define P_ON_M 0
#define P_ON_E 1
bool pOnE = true;
double initInput;
 
void Compute()
{
   if(!inAuto) return;
   unsigned long now = millis();
   int timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
      /*Compute all the working error variables*/
      double error = Setpoint - Input;
      ITerm+= (ki * error);
      if(ITerm > outMax) ITerm= outMax;
      else if(ITerm < outMin) ITerm= outMin;
      double dInput = (Input - lastInput);
  
      /*Compute P-Term*/
      if(pOnE) Output = kp * error;     ///改动
      else Output = -kp * (Input-initInput); ///改动
 
      /*Compute Rest of PID Output*/
      Output += ITerm - kd * dInput; ///改动
      if(Output > outMax) Output = outMax;
      else if(Output < outMin) Output = outMin;
  
      /*Remember some variables for next time*/
      lastInput = Input;
      lastTime = now;
   }
}
  
void SetTunings(double Kp, double Ki, double Kd, int pOn) ///改动
{
   if (Kp<0 || Ki<0|| Kd<0) return;
  
   pOnE = pOn == P_ON_E; ///改动
   
   double SampleTimeInSec = ((double)SampleTime)/1000;
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;
  
  if(controllerDirection ==REVERSE)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
}
  
void SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}
  
void SetOutputLimits(double Min, double Max)
{
   if(Min > Max) return;
   outMin = Min;
   outMax = Max;
  
   if(Output > outMax) Output = outMax;
   else if(Output < outMin) Output = outMin;
  
   if(ITerm > outMax) ITerm= outMax;
   else if(ITerm < outMin) ITerm= outMin;
}
  
void SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto == !inAuto)
    {  /*we just went from manual to auto*/
        Initialize();
    }
    inAuto = newAuto;
}
  
void Initialize()
{
   lastInput = Input;
   initInput = Input; ///改动
   ITerm = Output;
   if(ITerm > outMax) ITerm= outMax;
   else if(ITerm < outMin) ITerm= outMin;
}
  
void SetControllerDirection(int Direction)
{
   controllerDirection = Direction;
}
```

**注**：37，38，41，51，55，108为改动行。

随着输入的变化，“测量比例”会提供越来越大的阻碍，但是如果没有参考系，我们的性能会有些不稳定。 如果第一次打开控制器时PID输入为10000，我们真的要开始以Kp * 10000进行抵抗吗？ 否。我们希望将初始输入用作参考点（第108行），随着输入从此处更改（第38行），增加或减少阻力。
我们需要做的另一件事是允许用户选择是否要按比例进行误差或测量。 在上一篇文章之后，PonE似乎毫无用处，但重要的是要记住，对于许多循环来说，它运作良好。 因此，我们需要让用户选择他们想要的模式（第51和55行），然后在计算中采取相应的行动（第37和38行）。

###　第二遍修改---动态调整

尽管上面的代码确实可以工作，但是它有一个我们之前已经看到的问题。 在运行时更改调整参数时，会出现不希望出现的现象

![img](/images/PonM-blip.png)

为什么会这样呢?

![img](/images/PonM-blip-math-1.png)

我们上次看到此结果的原因是，积分是通过新的Ki重新缩放的。 这次是（Input – initInput）由Kp重新缩放。 我选择的解决方案类似于我对Ki所做的解决方案：我没有将Input – initInput视为一个整体单元乘以当前Kp的方法，而是将其分解为各个步骤，然后分别乘以Kp：

![img](/images/PonM-expansion.png)

```c
/*working variables*/
unsigned long lastTime;
double Input, Output, Setpoint;
double ITerm, lastInput;
double kp, ki, kd;
int SampleTime = 1000; //1 sec
double outMin, outMax;
bool inAuto = false;
  
#define MANUAL 0
#define AUTOMATIC 1
  
#define DIRECT 0
#define REVERSE 1
int controllerDirection = DIRECT;
  
#define P_ON_M 0
#define P_ON_E 1
bool pOnE = true;
double PTerm;
 
void Compute()
{
   if(!inAuto) return;
   unsigned long now = millis();
   int timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
    
      /*Compute all the working error variables*/      
      double error = Setpoint - Input;   
      ITerm+= (ki * error);  
      if(ITerm > outMax) ITerm= outMax;      
      else if(ITerm < outMin) ITerm= outMin;  
      double dInput = (Input - lastInput);
 
      /*Compute P-Term*/
      if(pOnE) Output = kp * error; 
      else 
      { 
         PTerm -= kp * dInput; 
         Output = PTerm; 
      } 
       
      /*Compute Rest of PID Output*/
      Output += ITerm - kd * dInput; 
    
      if(Output > outMax) Output = outMax;
      else if(Output < outMin) Output = outMin;
  
      /*Remember some variables for next time*/
      lastInput = Input;
      lastTime = now;
   }
}
  
void SetTunings(double Kp, double Ki, double Kd, int pOn)
{
   if (Kp<0 || Ki<0|| Kd<0) return;
  
   pOnE = pOn == P_ON_E;
   
   double SampleTimeInSec = ((double)SampleTime)/1000;
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;
  
  if(controllerDirection ==REVERSE)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
}
  
void SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}
  
void SetOutputLimits(double Min, double Max)
{
   if(Min > Max) return;
   outMin = Min;
   outMax = Max;
  
   if(Output > outMax) Output = outMax;
   else if(Output < outMin) Output = outMin;
  
   if(ITerm > outMax) ITerm= outMax;
   else if(ITerm < outMin) ITerm= outMin;
}
  
void SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto == !inAuto)
    {  /*we just went from manual to auto*/
        Initialize();
    }
    inAuto = newAuto;
}
  
void Initialize()
{
   lastInput = Input;
   PTerm = 0;
   ITerm = Output;
   if(ITerm > outMax) ITerm= outMax;
   else if(ITerm < outMin) ITerm= outMin;
}
  
void SetControllerDirection(int Direction)
{
   controllerDirection = Direction;
}
```

**注**:20,39-43,114为改动行

现在，我们保留一个有效的总和PTerm，而不是将Input-initInput的全部乘以Kp。 在每一步中，我们仅将当前输入更改乘以Kp并从PTerm中减去（第41行。）在这里，我们可以看到更改的影响：

![img](/images/PonM-no-blip.png)

![img](/images/PonM-no-blip-math.png)

由于旧的Kps位于“银行”中，因此调整参数的变化只会影响我们向前迈进
### 最后一遍–求和问题
关于上述代码的错误，我不会详细介绍（流行趋势等）。很好，但是仍然存在重大问题。例如：

1. 结束时间：虽然最终输出限制在outMin和outMax之间，但PTerm可能会在不希望的时候增加。它不会像完整的缠绕那样糟糕，但是仍然不能被接受
2. 即时更改：如果用户在运行时从P_ON_M更改为P_ON_E，则一段时间后返回，则不会对PTerm进行属性初始化，这会导致输出颠簸

还有更多，但仅这些就足以了解真正的问题是什么。在创建ITerm之前，我们已经处理了所有这些问题。我决定不为PTerm重新执行相同的解决方案，而是决定使用一种更美观的解决方案。
通过将PTerm和ITerm合并到一个称为“ outputSum”的变量中，P_ON_M代码将从已存在的所有ITerm修复中受益，并且由于代码中没有两个和，因此没有不必要的冗余。

```c
/*working variables*/
unsigned long lastTime;
double Input, Output, Setpoint;
double outputSum, lastInput;
double kp, ki, kd;
int SampleTime = 1000; //1 sec
double outMin, outMax;
bool inAuto = false;
  
#define MANUAL 0
#define AUTOMATIC 1
  
#define DIRECT 0
#define REVERSE 1
int controllerDirection = DIRECT;
  
#define P_ON_M 0
#define P_ON_E 1
bool pOnE = true;
 
 
void Compute()
{
   if(!inAuto) return;
   unsigned long now = millis();
   int timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
    
      /*Compute all the working error variables*/      
      double error = Setpoint - Input;   
      double dInput = (Input - lastInput);
      outputSum+= (ki * error);  
       
      /*Add Proportional on Measurement, if P_ON_M is specified*/
      if(!pOnE) outputSum-= kp * dInput
       
      if(outputSum > outMax) outputSum= outMax;      
      else if(outputSum < outMin) outputSum= outMin;  
     
      /*Add Proportional on Error, if P_ON_E is specified*/
      if(pOnE) Output = kp * error; 
      else Output = 0;
       
      /*Compute Rest of PID Output*/
      Output += outputSum - kd * dInput; 
    
      if(Output > outMax) Output = outMax;
      else if(Output < outMin) Output = outMin;
  
      /*Remember some variables for next time*/
      lastInput = Input;
      lastTime = now;
   }
}
  
void SetTunings(double Kp, double Ki, double Kd, int pOn)
{
   if (Kp<0 || Ki<0|| Kd<0) return;
  
   pOnE = pOn == P_ON_E;
   
   double SampleTimeInSec = ((double)SampleTime)/1000;
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;
  
  if(controllerDirection ==REVERSE)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
}
  
void SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}
  
void SetOutputLimits(double Min, double Max)
{
   if(Min > Max) return;
   outMin = Min;
   outMax = Max;
  
   if(Output > outMax) Output = outMax;
   else if(Output < outMin) Output = outMin;
  
   if(outputSum > outMax) outputSum= outMax;
   else if(outputSum < outMin) outputSum= outMin;
}
  
void SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto == !inAuto)
    {  /*we just went from manual to auto*/
        Initialize();
    }
    inAuto = newAuto;
}
  
void Initialize()
{
   lastInput = Input;
    
   outputSum = Output;
   if(outputSum > outMax) outputSum= outMax;
   else if(outputSum < outMin) outputSum= outMin;
}
  
void SetControllerDirection(int Direction)
{
   controllerDirection = Direction;
}
```

**注**:4,20,32,36,42,43,46,114为改动行.

那里有。 以上功能是Arduino PID v1.2.0中提供的功能。

### 但是，等等，还有更多：设定权重。
我没有在Arduino库代码中添加以下内容，但这是一项功能，如果您想自己动手，可能会感兴趣。 设定点加权是其核心，是同时拥有PonE和PonM的一种方法。 通过指定0到1之间的比率，您可以分别具有100％PonM，100％PonE或两者之间的某个比率。 如果您的流程集成度不理想（例如回流炉），并且对此有所考虑，这可能会有所帮助。
最终，我决定此时不将其添加到库中，因为它最终成为要调整/解释的另一个参数，而且我认为由此带来的好处是不值得的。 无论如何，这是代码，如果您想修改代码以具有设定值权重，而不仅仅是选择PonM / PonE：

```c
/*working variables*/
unsigned long lastTime;
double Input, Output, Setpoint;
double outputSum, lastInput;
double kp, ki, kd;
int SampleTime = 1000; //1 sec
double outMin, outMax;
bool inAuto = false;
  
#define MANUAL 0
#define AUTOMATIC 1
  
#define DIRECT 0
#define REVERSE 1
int controllerDirection = DIRECT;
  
#define P_ON_M 0
#define P_ON_E 1
bool pOnE = true, pOnM = false;
double pOnEKp, pOnMKp;
 
 
void Compute()
{
   if(!inAuto) return;
   unsigned long now = millis();
   int timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
    
      /*Compute all the working error variables*/      
      double error = Setpoint - Input;   
      double dInput = (Input - lastInput);
      outputSum+= (ki * error);  
       
      /*Add Proportional on Measurement, if P_ON_M is specified*/
      if(pOnM) outputSum-= pOnMKp * dInput
       
      if(outputSum > outMax) outputSum= outMax;      
      else if(outputSum < outMin) outputSum= outMin;  
     
      /*Add Proportional on Error, if P_ON_E is specified*/
      if(pOnE) Output = pOnEKp * error; 
      else Output = 0;
       
      /*Compute Rest of PID Output*/
      Output += outputSum - kd * dInput; 
    
      if(Output > outMax) Output = outMax;
      else if(Output < outMin) Output = outMin;
  
      /*Remember some variables for next time*/
      lastInput = Input;
      lastTime = now;
   }
}
  
void SetTunings(double Kp, double Ki, double Kd, double pOn)
{
   if (Kp<0 || Ki<0|| Kd<0 || pOn<0 || pOn>1) return;
  
   pOnE = pOn>0; //some p on error is desired;
   pOnM = pOn<1; //some p on measurement is desired;  
   
   double SampleTimeInSec = ((double)SampleTime)/1000;
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;
  
  if(controllerDirection ==REVERSE)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
    
   pOnEKp = pOn * kp; 
   pOnMKp = (1 - pOn) * kp;
}
  
void SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}
  
void SetOutputLimits(double Min, double Max)
{
   if(Min > Max) return;
   outMin = Min;
   outMax = Max;
  
   if(Output > outMax) Output = outMax;
   else if(Output < outMin) Output = outMin;
  
   if(outputSum > outMax) outputSum= outMax;
   else if(outputSum < outMin) outputSum= outMin;
}
  
void SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto == !inAuto)
    {  /*we just went from manual to auto*/
        Initialize();
    }
    inAuto = newAuto;
}
  
void Initialize()
{
   lastInput = Input;
   outputSum = Output;
   if(outputSum > outMax) outputSum= outMax;
   else if(outputSum < outMin) outputSum= outMin;
}
  
void SetControllerDirection(int Direction)
{
   controllerDirection = Direction;
}
```

**注**:19,20,37,43,58,60,62,63,77,78为改动行.

现在，pOn不再是将pOn设置为整数，而是以允许允许比率的双精度形式出现（第58行）。除了一些标志（第62和63行）以外，还在第77-78行计算了加权Kp项。 然后，在第37和43行上，将加权的PonM和PonE贡献添加到总体PID输出中。