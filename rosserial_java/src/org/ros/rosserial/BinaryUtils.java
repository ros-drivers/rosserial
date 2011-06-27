// Software License Agreement (BSD License)
//
// Copyright (c) 2011, Willow Garage, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of Willow Garage, Inc. nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

package org.ros.rosserial;

public class BinaryUtils {

	//Converts a Byte array into a string representation
	//Useful for pretty printing hex
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
