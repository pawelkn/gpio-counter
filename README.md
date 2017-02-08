# gpio-counter

Generic GPIO impulse counter. Counts impulses using GPIO interrupts.
Can be activated on low or high level. If needed, 
a debounce procedure can be performed.

## Cross-compile build
```sh
export PATH=<path to toolchain executables>
export KERNEL_DIR=<path to kernel build directory>
export CROSS_COMPILE=<cross compilation prefix>
export ARCH=<architecture eg.: arm>
make
```

## Device tree bindings

Required properties:
* gpios: a spec for a GPIO to be used

Optional properties:
* debounce-delay-ms: pulse debounce duration

Example:
```c
        counter {
                compatible = "gpio-counter";
                gpios = <&gpio1 14 GPIO_ACTIVE_HIGH>;
                debounce-delay-ms = <1>;
        };
```
