package org.firstinspires.ftc.teamcode.libs;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.TimestampedData;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.util.Arrays;

@I2cDeviceType
@DeviceProperties(name = "PixyCam2", description = "PixyCam2.1 on the I2C Bus", xmlTag = "PIXYCAM2")
public class PixyCam2 extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x54); // TODO: This could be a parameter

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Methods for use in Op Modes
    //
    // Note: To get an instance of this class, use the FTC hardwareMap as follows:
    //       PixyCam2 pixyCam = hardwareMap.get(PixyCam2.class, "pixycam-name");
    //
    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    public boolean getVersionInfo() {
        if (!versionInfoRetrieved) {
            sendSimpleRequest(VERSION_INFO_REQUEST); // PixyCam should be ready to read results from in < 100 microseconds!
            return readVersionInfo();
        }
        return true;
    }
    public short getHWVersion() { return HWVersion; }
    public short getFWVersionMajor() { return FWVersionMajor; }
    public short getFWVersionMinor() { return FWVersionMinor; }
    public short getBuildNum() { return FWBuildNum; }
    public String getFWType () { return FWType; };

    public void setLEDs(byte r, byte g, byte b) {
        byte[] request = {
                (byte)0xae,  // first two bytes specify no check-sum data
                (byte)0xc1,
                (byte) 20,  // the code for the request type
                (byte) 3,
                r,
                g,
                b
        };
        if (logDetails) teamUtil.log("PixyCam: Set LEDs:"+ Arrays.toString(request));

        deviceClient.write(request);
    }

    public void toggleLEDs(boolean upper, boolean lower) {
        byte[] request = {
                (byte)0xae,  // first two bytes specify no check-sum data
                (byte)0xc1,
                (byte) 22,  // the code for the request type
                (byte) 2,
                upper ? (byte)1 : (byte)0, // upper leds
                lower ? (byte)1 : (byte)0, // lower led
        };
        if (logDetails) teamUtil.log("PixyCam: Toggle LEDs:"+ Arrays.toString(request));

        deviceClient.write(request);
    }

    public PixyBlock[] getBlocks(byte signatures, byte numBlocks) {
        if (numBlocks > 6) {
            teamUtil.log("ERROR : PixyCam Get Blocks request for more than 6 blocks.  Exceeds I2C request size");
            PixyBlock[] blocks = {};
            return blocks;
        }
        byte[] request = {  (byte)0xae,  // first two bytes specify no check-sum data
                (byte)0xc1,
                BLOCKS_REQUEST,  // the code for the request type
                (byte)2, // 2 bytes of payload data
                signatures,
                numBlocks};
        if (logDetails) teamUtil.log("PixyCam: Send Blocks Request:"+ Arrays.toString(request));
        deviceClient.write(request);
        //debugResponse(HEADER_LENGTH+numBlocks*BLOCKS_PAYLOAD_SIZE);
        byte[] response = readResponse2(BLOCKS_RESPONSE,HEADER_LENGTH+numBlocks*BLOCKS_PAYLOAD_SIZE);
        teamUtil.log(""+response);
        if (response == null) {
            PixyBlock[] blocks = {};
            return blocks;
        } else {
            PixyBlock[] blocks = new PixyBlock[response.length / 14];
            for (int i = 0; i < response.length / 14; i++) {
                blocks[i] = new PixyBlock();
                blocks[i].signature = TypeConversion.byteArrayToShort(response, i * 14, ByteOrder.LITTLE_ENDIAN);
                blocks[i].xCenter = TypeConversion.byteArrayToShort(response, i * 14 + 2, ByteOrder.LITTLE_ENDIAN);
                blocks[i].yCenter = TypeConversion.byteArrayToShort(response, i * 14 + 4, ByteOrder.LITTLE_ENDIAN);
                blocks[i].width = TypeConversion.byteArrayToShort(response, i * 14 + 6, ByteOrder.LITTLE_ENDIAN);
                blocks[i].height = TypeConversion.byteArrayToShort(response, i * 14 + 8, ByteOrder.LITTLE_ENDIAN);
                blocks[i].angle = TypeConversion.byteArrayToShort(response, i * 14 + 10, ByteOrder.LITTLE_ENDIAN);
                blocks[i].index = response[i * 14 + 12];
                blocks[i].age = response[i * 14 + 13];
            }
            return blocks;
        }
    }
    public void showBug() {
        sendSimpleRequest(VERSION_INFO_REQUEST);
        debugResponse(100);
        sendSimpleRequest(VERSION_INFO_REQUEST);
        debugResponse(4);
        debugResponse(18);
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Internal Helper Methods
    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    private boolean logDetails = true;
    private  static final int HEADER_LENGTH = 6;
    private static final byte VERSION_INFO_REQUEST = 0x0e; // Request ID for Version Information from PixyCam
    private static final byte VERSION_INFO_RESPONSE = 0x0f; // Response ID for Version Information from PixyCam
    private static final int VERSION_INFO_PAYLOAD_SIZE = 16; // Payload data size for version info

    private static final byte BLOCKS_REQUEST = (byte) 32;
    private static final byte BLOCKS_RESPONSE = (byte) 33;
    private static final int BLOCKS_PAYLOAD_SIZE = 14;

    private boolean versionInfoRetrieved = false;
    private short HWVersion, FWVersionMajor, FWVersionMinor, FWBuildNum;
    private String FWType = "";





    private boolean readVersionInfo() {
        if (logDetails) teamUtil.log("PixyCam: Reading Version Info");

        byte[] rawVersionInfo = readResponse2(VERSION_INFO_RESPONSE, HEADER_LENGTH+VERSION_INFO_PAYLOAD_SIZE);
        if (logDetails) teamUtil.log("PixyCam: Returned Data: "+ Arrays.toString(rawVersionInfo));

        if (rawVersionInfo == null || rawVersionInfo.length != VERSION_INFO_PAYLOAD_SIZE) {
            // failed to get data back
            if (logDetails) teamUtil.log("PixyCam: Bad Version Data");
            return false;
        } else {
            versionInfoRetrieved = true;
            HWVersion = TypeConversion.byteArrayToShort(rawVersionInfo,0, ByteOrder.LITTLE_ENDIAN);
            FWVersionMajor = (short) rawVersionInfo[2];
            FWVersionMinor = (short) rawVersionInfo[3];
            FWBuildNum = TypeConversion.byteArrayToShort(rawVersionInfo,4, ByteOrder.LITTLE_ENDIAN);

            // Convert the rest of the byte array into a string
            int strLength = 0;
            while (rawVersionInfo[6+strLength] != 0 && strLength < 10) { // find end of string
                strLength++;
            }
            FWType = new String(rawVersionInfo, 6, strLength, StandardCharsets.UTF_8);
            return true;
        }

    }

    // Send a simple 4 byte request to PixyCam with the specified request type code
    private void sendSimpleRequest (byte type) {
        byte[] request = {  (byte)0xae,  // first two bytes specify no check-sum data
                (byte)0xc1,
                type,  // the code for the request type
                (byte)0x00}; // no extra data included in this request
        if (logDetails) teamUtil.log("PixyCam: Send Request:"+ Arrays.toString(request));

        deviceClient.write(request);
    }



    private void debugResponse(int bytes) {
        byte[] result = deviceClient.read(bytes);
        teamUtil.log("debugResponse:" + Arrays.toString(result));
    }
    private void debugResponseTS(int bytes) {
        TimestampedData result = deviceClient.readTimeStamped(bytes);
        teamUtil.log("debugResponse:" + Arrays.toString(result.data));
    }



    // Read a response from PixyCam as a single read operation, validate the response type and return the payload data
    private byte[] readResponse2(byte expectedResponseType, int expectedResponseSize) {
        if (logDetails) teamUtil.log("PixyCam: Read Response2");

        byte[] response = deviceClient.read(expectedResponseSize);
        if (logDetails) teamUtil.log("PixyCam: Read Response :"+ Arrays.toString(response));

        if (response.length < expectedResponseSize ) {
            if (logDetails) teamUtil.log("PixyCam: Read Response less than expected");
            return null;
        } else if (response[2] != expectedResponseType) {
            // error: unexpected response type
            if (logDetails) teamUtil.log("PixyCam: Response Code Mismatch:" + expectedResponseType +":" + response[2]);
            return null;
        }

        //if (validateCheckSum(response)) {
        return Arrays.copyOfRange(response, HEADER_LENGTH, HEADER_LENGTH+response[3]); // copy out data payload without header and extraneous bytes
        //} else {
        //   return null;
        //}
    }

    // Read a response from PixyCam, validate the response type and return the payload data
    // WARNING!! This code doesn't work due to a bug in the rev hub firmware where subsequent reads of the same I2C synch response will be missing the first byte
    /*
    private byte[] readResponse(byte expectedResponseType) {
        if (logDetails) teamUtil.log("PixyCam: Read Response");

        byte[] headerResult = deviceClient.read(HEADER_LENGTH);
        if (logDetails) teamUtil.log("PixyCam: Read Response Header:"+ Arrays.toString(headerResult));

        if (headerResult.length < 4 ) {
            // error: didn't get at least 4 bytes back
            if (logDetails) teamUtil.log("PixyCam: Read Response < 4 bytes");
            return null;
        } else if (headerResult[2] != expectedResponseType) {
            // error: unexpected response type
            if (logDetails) teamUtil.log("PixyCam: Response Code Mismatch:" + expectedResponseType +":" + headerResult[2]);
            return null;
        }
        if (headerResult[0] == (byte) 0xaf) { // checksum data included
            byte[] checkSumResult = deviceClient.read(2); // read the first two bytes for check sum
            short checkSum = TypeConversion.byteArrayToShort(checkSumResult, ByteOrder.LITTLE_ENDIAN);
            if (logDetails) teamUtil.log("PixyCam: Response Checksum:" + Arrays.toString(checkSumResult)+ ": " + checkSum);
            byte[] payloadData = deviceClient.read(headerResult[3]); // read the data payload
            // Validate checksum
            return payloadData;

        } else {
            return deviceClient.read(headerResult[3]); // read and return the data payload
        }
    }
*/

    private boolean validateCheckSum(byte[] response) {
        short checkSum = TypeConversion.byteArrayToShort(response,4, ByteOrder.LITTLE_ENDIAN);
        if (logDetails) teamUtil.log("PixyCam: Response Checksum:" + checkSum);
        int sum = 0;
        for (int i=HEADER_LENGTH;i<response.length;i++) {
            sum+= response[i];
        }
        if (logDetails) teamUtil.log("PixyCam: Response Computed Checksum:" + sum);

        if (checkSum == sum) {
            return true;
        } else {
            teamUtil.log("PixyCam: Invalid Checksum on Response: "+checkSum+" : "+ sum);
            return false;
        }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Methods called by the FTC SDK
    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    public PixyCam2(I2cDeviceSynch deviceClient)
    {
        super(deviceClient, true);

        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);

        super.registerArmingStateCallback(false); // Deals with USB cables getting unplugged

        // Sensor starts off disengaged so we can change things like I2C address. Need to engage
        this.deviceClient.engage();
    }

    @Override
    protected synchronized boolean doInitialize()
    {
        return true;
    }

    @Override
    public Manufacturer getManufacturer()
    {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName()
    {
        return "PixyCam 2.1";
    }
}

