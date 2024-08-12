/*
 * PROJECT:         ReactOS HAL
 * LICENSE:         GPL - See COPYING in the top level directory
 * FILE:            hal/halx86/apic/tsc.c
 * PURPOSE:         HAL Routines for TSC handling
 * PROGRAMMERS:     Timo Kreuzer (timo.kreuzer@reactos.org)
 */

/* INCLUDES ******************************************************************/

#include <hal.h>
#include "tsc.h"
#include "apicp.h"
#define NDEBUG
#include <debug.h>

LARGE_INTEGER HalpCpuClockFrequency = {{INITIAL_STALL_COUNT * 1000000}};

UCHAR TscCalibrationPhase;
ULONG64 TscCalibrationArray[NUM_SAMPLES];

#define RTC_MODE 6 /* Mode 6 is 1024 Hz */
#define SAMPLE_FREQUENCY ((32768 << 1) >> RTC_MODE)

/* PRIVATE FUNCTIONS *********************************************************/

static
ULONG64
DoLinearRegression(
    ULONG XMax,
    ULONG64 *ArrayY)
{
    ULONG X, SumXX;
    ULONG64 SumXY;

    /* Calculate the sum of the squares of X */
    SumXX = (XMax * (XMax + 1) * (2*XMax + 1)) / 6;

    /* Calculate the sum of the differences to the first value
       weighted by x */
    for (SumXY = 0, X = 1; X <= XMax; X++)
    {
         SumXY += X * (ArrayY[X] - ArrayY[0]);
    }

    /* Account for sample frequency */
    SumXY *= SAMPLE_FREQUENCY;

    /* Return the quotient of the sums */
    return (SumXY + (SumXX/2)) / SumXX;
}

#include "Mile.HyperV/Mile.HyperV.VMBus.h"
#include "apicp.h"

FORCEINLINE
ULONG
TestedIOApicRead(UCHAR Register)
{
    /* Select the register, then do the read */
    ASSERT(Register <= 0x3F);
    WRITE_REGISTER_ULONG((PULONG)(IOAPIC_BASE + IOAPIC_IOREGSEL), Register);
    return READ_REGISTER_ULONG((PULONG)(IOAPIC_BASE + IOAPIC_IOWIN));
}

FORCEINLINE
IOAPIC_REDIRECTION_REGISTER
TestedApicReadIORedirectionEntry(
    UCHAR Index)
{
    IOAPIC_REDIRECTION_REGISTER ReDirReg;

    ASSERT(Index < APIC_MAX_IRQ);
    ReDirReg.Long0 = TestedIOApicRead(IOAPIC_REDTBL + 2 * Index);
    ReDirReg.Long1 = TestedIOApicRead(IOAPIC_REDTBL + 2 * Index + 1);

    return ReDirReg;
}

VOID
NTAPI
HalpInitializeTsc(VOID)
{
    ULONG_PTR Flags;
    PVOID PreviousHandler;
    UCHAR RegisterA, RegisterB;

    /* Check if the CPU supports RDTSC */
    if (!(KeGetCurrentPrcb()->FeatureBits & KF_RDTSC))
    {
        KeBugCheck(HAL_INITIALIZATION_FAILED);
    }

    {
        /* Set clock multiplier to 1 */
        ApicWrite(APIC_TDCR, TIMER_DV_DivideBy1);

        /* Set APIC init counter to -1 */
        ApicWrite(APIC_TICR, (ULONG)0xFFFFFFFF);

        LVT_REGISTER LvtEntry;

        /* Set to periodic */
        LvtEntry.Long = 0;
        LvtEntry.TimerMode = 1;
        LvtEntry.Vector = APIC_CLOCK_VECTOR;
        LvtEntry.Mask = 0;
        ApicWrite(APIC_TMRLVTR, LvtEntry.Long);
    }

     /* Save flags and disable interrupts */
    Flags = __readeflags();
    _disable();

    /* Enable the periodic interrupt in the CMOS */
    RegisterB = HalpReadCmos(RTC_REGISTER_B);
    HalpWriteCmos(RTC_REGISTER_B, RegisterB | RTC_REG_B_PI);

    /* Modify register A to RTC_MODE to get SAMPLE_FREQUENCY */
    RegisterA = HalpReadCmos(RTC_REGISTER_A);
    RegisterA = (RegisterA & 0xF0) | RTC_MODE;
    HalpWriteCmos(RTC_REGISTER_A, RegisterA);

    /* Save old IDT entry */
    PreviousHandler = KeQueryInterruptHandler(APIC_CLOCK_VECTOR);

    /* Set the calibration ISR */
    KeRegisterInterruptHandler(APIC_CLOCK_VECTOR, TscCalibrationISR);

    /* Reset TSC value to 0 */
    __writemsr(MSR_RDTSC, 0);

    /* Enable the timer interrupt */
    HalEnableSystemInterrupt(APIC_CLOCK_VECTOR, CLOCK_LEVEL, Latched);

    /* Read register C, so that the next interrupt can happen */
    HalpReadCmos(RTC_REGISTER_C);

    DPRINT1("Yolo %llX\n", __readmsr(MSR_RDTSC));

    IOAPIC_REDIRECTION_REGISTER ReDirReg;
    ReDirReg = TestedApicReadIORedirectionEntry(APIC_CLOCK_INDEX);
    DPRINT1("IOAPIC_REDIRECTION_REGISTER Index APIC_CLOCK_INDEX: LongLong %llX, Mask: %llX\n", ReDirReg.LongLong, ReDirReg.Mask);

    DPRINT1("[Kenji Mouri] Yolo Life La La Land Prelude\n");

    /* Wait for completion */
    //_enable();
    //while (TscCalibrationPhase < NUM_SAMPLES) _ReadWriteBarrier();
    //_disable();

    DPRINT1("[Kenji Mouri] Yolo Life La La Land Postlude\n");

    /* Disable the periodic interrupt in the CMOS */
    HalpWriteCmos(RTC_REGISTER_B, RegisterB & ~RTC_REG_B_PI);

    /* Disable the timer interrupt */
    HalDisableSystemInterrupt(APIC_CLOCK_VECTOR, CLOCK_LEVEL);

    /* Restore the previous handler */
    KeRegisterInterruptHandler(APIC_CLOCK_VECTOR, PreviousHandler);

    /* Calculate an average, using simplified linear regression */
    HalpCpuClockFrequency.QuadPart = DoLinearRegression(NUM_SAMPLES - 1,
                                                        TscCalibrationArray);

    {
        DPRINT1("HV_X64_MSR_TSC_FREQUENCY = %lld\n", __readmsr(HV_X64_MSR_TSC_FREQUENCY));
        DPRINT1("HV_X64_MSR_APIC_FREQUENCY = %lld\n", __readmsr(HV_X64_MSR_APIC_FREQUENCY));

        ULONGLONG TscFrequency = __readmsr(HV_X64_MSR_TSC_FREQUENCY);
        ULONG ApicTimerFrequency = __readmsr(HV_X64_MSR_APIC_FREQUENCY);

        DPRINT1("Current TSC Frequency = %lld\n", TscFrequency);
        DPRINT1("ApicTimerFrequency = %lld\n", ApicTimerFrequency);

        //DbgBreakPoint();

        LVT_REGISTER LvtEntry;
        ULONGLONG TimerInterval;

        /* Calculate the Timer interval */
        TimerInterval = ApicTimerFrequency / 1000;

        HalpCpuClockFrequency.QuadPart = TscFrequency;
        DPRINT1("HalpCpuClockFrequency.QuadPart %llX\n", HalpCpuClockFrequency.QuadPart);

        /* Set the count interval */
        ApicWrite(APIC_TICR, (ULONG)TimerInterval);

        /* Set to periodic */
        LvtEntry.Long = 0;
        LvtEntry.TimerMode = 1;
        LvtEntry.Vector = APIC_CLOCK_VECTOR;
        LvtEntry.Mask = 0;
        ApicWrite(APIC_TMRLVTR, LvtEntry.Long);

        DPRINT1("[Kenji Mouri] I start to yolo again.\n");
    }

    /* Restore flags */
    __writeeflags(Flags);

}

VOID
NTAPI
HalpCalibrateStallExecution(VOID)
{
    // Timer interrupt is now active

    HalpInitializeTsc();

    KeGetPcr()->StallScaleFactor = (ULONG)(HalpCpuClockFrequency.QuadPart / 1000000);
}

/* PUBLIC FUNCTIONS ***********************************************************/

LARGE_INTEGER
NTAPI
KeQueryPerformanceCounter(
    OUT PLARGE_INTEGER PerformanceFrequency OPTIONAL)
{
    LARGE_INTEGER Result;

    /* Make sure it's calibrated */
    ASSERT(HalpCpuClockFrequency.QuadPart != 0);

    /* Does the caller want the frequency? */
    if (PerformanceFrequency)
    {
        /* Return tsc frequency */
        *PerformanceFrequency = HalpCpuClockFrequency;
    }

    /* Return the current value */
    Result.QuadPart = __rdtsc();
    return Result;
}

VOID
NTAPI
KeStallExecutionProcessor(ULONG MicroSeconds)
{
    ULONG64 StartTime, EndTime;

    /* Get the initial time */
    StartTime = __rdtsc();

    /* Calculate the ending time */
    EndTime = StartTime + KeGetPcr()->StallScaleFactor * MicroSeconds;

    /* Loop until time is elapsed */
    while (__rdtsc() < EndTime);
}

VOID
NTAPI
HalCalibratePerformanceCounter(
    IN volatile PLONG Count,
    IN ULONGLONG NewCount)
{
    UNIMPLEMENTED;
    ASSERT(FALSE);
}

