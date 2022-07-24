// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.GeneralPin;
import com.ctre.phoenix.CANifierStatusFrame;

// Implementation based off https://www.ti.com/lit/an/slva704/slva704.pdf
public class CanifierI2C {
    private final CANifier canifier;

    private final boolean HIGH = true;
    private final boolean LOW = false; 

    private final long I2C_DELAY_NS = TimeUnit.MILLISECONDS.toNanos(10);
    private final long SLEEP_PRECISION = TimeUnit.MILLISECONDS.toNanos(2);

    // Bitmask is LSB -> MSB of GeneralPin index
    // 0 0 0 0 1 1 0 0 0 0 0
    //    SCL--^ ^-- SDA
    private final int SDA_MASK = 0x20;
    private final int SCL_MASK = 0x40; 

    public CanifierI2C(int deviceId) {
        canifier = new CANifier(deviceId);

        // Since the status frame period of the CANifier is 10ms (100Hz)
        // For Digital Input/Output by default, we can use this instead
        canifier.setStatusFramePeriod(CANifierStatusFrame.Status_2_General, 10);
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

    // Sets SDA/SCL at the same time
    // It is expensive to call setGeneralOutput twice
    // If my understanding is correct, though, It should be fine
    // To call the original methods if setting individually (same time?)
    private void batchSet(boolean sdaState, boolean sclState) {
        int mask = 0x0;
        if(sdaState)
            mask |= SDA_MASK;
        if(sclState)
            mask |= SCL_MASK;
        canifier.setGeneralOutputs(mask, mask);
    }

    private void delay() {
        long start = System.nanoTime();
        long end = 0;
        do {
            end = System.nanoTime();
        } while (start + I2C_DELAY_NS >= end);
    }

    // Start sequence: Pull both high for one cycle, then put SDA down.
    private synchronized void startCondition() {
        // Pull both high
        // setSDA(HIGH);
        // setSCL(HIGH);

        batchSet(HIGH, HIGH);

        delay();

        // After 1/2 cycle, pull SDA low
        setSDA(LOW);

        delay();
        setSCL(LOW);
    }

    // Stop sequence: Keep SCL High and pull up SDA after one cycle.
    private synchronized void stopCondition() {
        setSDA(LOW);
        delay();
        setSCL(HIGH);
        
        delay();
        setSDA(HIGH);

        delay();
    }

    // Apparently you can only shift ints...
    private synchronized boolean writeByte(int data) {
        for(int i=0;i<8;i++) {
            setSDA((data & 0x80) == 1); // Mask for eighth bit
            data = data << 1;
            delay();
            setSCL(HIGH);
            delay();
            setSCL(LOW);
        }

        // read that ACK bit!!!!
        batchSet(HIGH, HIGH);
        // setSDA(HIGH);
        // setSCL(HIGH);
        delay();
        
        // This ideally should be instantaneous
        // But if my previous assumptions are correct about latency
        // FIXME: this probably doesn't work
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
            delay();

            // Set last bit
            if(getSDA()) data |= 1;

            delay();
            setSCL(LOW);
        }
        
        // set ACK/NACK
        batchSet(!ack, HIGH);
        // setSDA(!ack);
        // setSCL(HIGH);
        
        delay();
        batchSet(HIGH, LOW);
        // setSDA(HIGH);
        // setSCL(LOW);
        
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

        stopCondition();

        return true;
    }
}