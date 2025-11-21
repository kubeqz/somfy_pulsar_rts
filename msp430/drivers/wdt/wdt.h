#ifndef DRIVERS_WDT_WDT_H
#define DRIVERS_WDT_WDT_H

class Wdt
{
public:
    enum WdtTimeout {
        WdtTimeoutFast,
        WdtTimeoutSlow
    };

    void enable(WdtTimeout);

    void disable();
};

#endif /* DRIVERS_WDT_WDT_H */
