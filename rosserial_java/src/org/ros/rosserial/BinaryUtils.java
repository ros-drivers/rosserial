package org.ros.tutorials.rjs;

public class BinaryUtils {

	static String byteArrayToHexString(byte in[]) {

    byte ch = 0x00;

    int i = 0; 

    if (in == null || in.length <= 0)

        return null;

        

    String pseudo[] = {"0", "1", "2",
						"3", "4", "5", "6", "7", "8",
						"9", "A", "B", "C", "D", "E",
						"F"};

    StringBuffer out = new StringBuffer(in.length * 5);

    

    while (i < in.length) {
		out.append("0x");
        ch = (byte) (in[i] & 0xF0); // Strip off high nibble

        ch = (byte) (ch >>> 4); // shift the bits down

        ch = (byte) (ch & 0x0F);    // must do this is high order bit is on!

        out.append(pseudo[ (int) ch]); // convert thenibble to a String Character

        ch = (byte) (in[i] & 0x0F); // Strip offlow nibble 

        out.append(pseudo[ (int) ch]); // convert thenibble to a String Character
        out.append(" " );

        i++;

    }

    String rslt = new String(out);

    return rslt;

}   
}
