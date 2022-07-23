// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.GeneralPin;

// Implementation based off https://www.ti.com/lit/an/slva704/slva704.pdf

public class CanifierI2C {
    private final CANifier canifier;

    private final boolean HIGH = true;
    private final boolean LOW = false; 

    // 5 us delay
    private final long I2C_DELAY_NS = 5000;
    private final long SLEEP_PRECISION = TimeUnit.MILLISECONDS.toNanos(2);

    public CanifierI2C(int deviceId) {
        canifier = new CANifier(deviceId);
    }

    private void setSCL(boolean val) {
        canifier.setGeneralOutput(GeneralPin.SCL, val, true);
    }

    private void setSDA(boolean val) {
        canifier.setGeneralOutput(GeneralPin.SDA, val, true);
    }

    private boolean getSCL() {
        return canifier.getGeneralInput(GeneralPin.SCL);
    }

    private boolean getSDA() {
        return canifier.getGeneralInput(GeneralPin.SDA);
    }

    // sleep function I found on stack overflow, since I'm scared of Thread.sleep()
    // and its apparent variability.
    private void delay() throws InterruptedException {
        final long end = System.nanoTime() + I2C_DELAY_NS;
        long timeLeft = I2C_DELAY_NS;
        do {
            if (timeLeft > SLEEP_PRECISION)
                Thread.sleep(1);
            else
                Thread.sleep(0);
            timeLeft = end - System.nanoTime();
        
            if(Thread.interrupted())
                throw new InterruptedException();
        } while (timeLeft > 0);
    }

    // Start sequence: Pull both high for one cycle, then put SDA down.
    private synchronized void startCondition() {
        // Pull both high
        setSDA(HIGH);
        setSCL(HIGH);

        try { delay(); } catch(InterruptedException e) {}

        // After 5 us pull SDA low
        setSDA(LOW);

        try { delay(); } catch(InterruptedException e) {}
        setSCL(LOW);
    }

    // Stop sequence: Keep SCL High and pull up SDA after one cycle.
    private synchronized void stopCondition() {
        setSDA(LOW);
        try { delay(); } catch(InterruptedException e) {}
        setSCL(HIGH);
        
        try { delay(); } catch(InterruptedException e) {}
        setSDA(HIGH);

        try { delay(); } catch(InterruptedException e) {}
    }

    // Apparently you can only shift ints...
    private synchronized boolean writeByte(int data) {
        for(int i=0;i<8;i++) {
            setSDA((data & 0x80) == 1); // Mask for eighth bit
            data = data << 1;
            try { delay(); } catch(InterruptedException e) {}
            setSCL(HIGH);
            try { delay(); } catch(InterruptedException e) {}
            setSCL(LOW);
        }

        // read that ACK bit!!!!
        setSDA(HIGH);
        setSCL(HIGH);
        try { delay(); } catch(InterruptedException e) {}
        boolean ack = !getSDA();
        setSCL(LOW);

        return ack;
    }

    // Read one byte, write ACK only if needed
    private synchronized int readByte(boolean ack) {
        int data = 0;

        // read each bit
        setSDA(HIGH);
        for(int i=0;i<8;i++) {
            data = data << 1;

            // clock stretching
            do {
                setSCL(HIGH);
            } while(getSCL() == LOW);
            try { delay(); } catch(InterruptedException e) {}

            // Set last bit
            if(getSDA()) data |= 1;

            try { delay(); } catch(InterruptedException e) {}
            setSCL(LOW);
        }
        
        // ACK bit set to high
        setSDA(!ack);
        setSCL(HIGH);
        try { delay(); } catch(InterruptedException e) {}
        setSCL(LOW);
        setSDA(HIGH);
        return data;
    }

    public boolean write(int address, int register, int data) {
        startCondition();

        // Write slave address. Set R/W bit to write
        if(!writeByte((address << 1) | 0x00)) 
            return false;

        // Write register address
        if(!writeByte(register)) 
            return false; 

        // Write slave address. Set R/W bit to write
        if(!writeByte(data)) 
            return false; 

        stopCondition();

        return true;
    }

    public boolean read(int address, int register, int numBytes, byte[] data) {
        startCondition();

        // Write slave address. Set R/W bit to read
        if(!writeByte((address << 1) | 0x1))
            return false;

        if(!writeByte(register))
            return false;
        
        // only write NACK when done reading
        for(int i=0;i<numBytes;i++) {
            data[i] = (i == numBytes-1) ? (byte)readByte(true) : (byte)readByte(false);
        }

        return true;
    }
}