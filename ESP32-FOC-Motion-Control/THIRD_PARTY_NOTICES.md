# 第三方依赖与代码来源

本仓库的主要代码为应用层控制程序；底层 FOC 依赖外部 **SimpleFOC 2.2.1** 库，控制板基于 **DengFOC-V4** 硬件及其参考工程。

- [SimpleFOC / Arduino-FOC](https://github.com/simplefoc/Arduino-FOC)：由其作者和贡献者维护，单独安装，不随本仓库打包。
- [DengFOC / Deng's FOC Controller](https://github.com/ToanTech/Deng-s-foc-controller)：提供硬件平台和部分参考实现基础，感谢原作者及社区贡献者。

**发布前注意：** 请依据最初采用的 DengFOC 示例文件逐段确认哪些代码属于复制/改编，核对原始版权声明及许可证要求，并在必要时保留原版权和许可文本。致谢文字不能代替原许可证规定的义务。本仓库当前未统一声明 MIT/GPL 等开源许可证，不应将引用或改编的内容全部标为独立原创。
