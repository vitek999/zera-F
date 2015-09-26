/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 *
 * MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

package com.mediatek.dm.xml;

import android.text.TextUtils;
import android.util.Log;

import com.mediatek.dm.DmConst;
import com.mediatek.dm.DmConst.TAG;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.transform.Transformer;
import javax.xml.transform.TransformerException;
import javax.xml.transform.TransformerFactory;
import javax.xml.transform.dom.DOMSource;
import javax.xml.transform.stream.StreamResult;

import org.w3c.dom.Document;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

public class DmXMLParser {
    private Document mDocument;
    private boolean mIsParseSucceed;
    private String mFileName;

    public DmXMLParser(String filename) {
        mFileName = filename;
        Log.i(TAG.XML, "DmXMLParser parse file " + filename);
        mIsParseSucceed = parse(filename);
    }

    boolean parse(String fileName) {
        DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
        try {
            DocumentBuilder builder = factory.newDocumentBuilder();
            File configFile = new File(fileName);
            mDocument = builder.parse(configFile);
        } catch (ParserConfigurationException e) {
            e.printStackTrace();
            return false;
        } catch (SAXException e) {
            e.printStackTrace();
            return false;
        } catch (IOException e) {
            e.printStackTrace();
            return false;
        }
        return true;
    }

    public String getValByTagName(String tagName) {
        NodeList dmValueNodeList = mDocument.getElementsByTagName(tagName);
        if (dmValueNodeList == null) {
            throw new IllegalArgumentException("getElementsByTagName failed for " + tagName);
        }

        Node dmValueNode = dmValueNodeList.item(0);
        return dmValueNode.getTextContent();
    }

    public void setValByTagName(String tagName, String value) {
        NodeList dmValueNodeList = mDocument.getElementsByTagName(tagName);
        if (dmValueNodeList == null) {
            throw new IllegalArgumentException("getElementsByTagName failed for " + tagName);
        }

        Node dmValueNode = dmValueNodeList.item(0);
        dmValueNode.setTextContent(value);
    }

    // If modify the content of document, should call this API to write back
    public void writeBack() {
        try {
            TransformerFactory tf = TransformerFactory.newInstance();
            Transformer transformer = tf.newTransformer();
            DOMSource source = new DOMSource(mDocument);
            StreamResult result = new StreamResult(new File(mFileName));
            transformer.transform(source, result);
        } catch (TransformerException e) {
            e.printStackTrace();
        }
    }

    public void getChildNode(Node node, List<Node> nodeList, String... nodeName) {
        if (!mIsParseSucceed) {
            return;
        }

        if (nodeName.length == 0) {
            getChildNodeList(node, null, nodeList, false);
        } else {
            getChildNodeList(node, nodeName[0], nodeList, false);
        }
        return;
    }

    public void getChildNode(List<Node> nodeList, String... nodeName) {
        if (!mIsParseSucceed) {
            return;
        }

        if (nodeName.length == 0) {
            getChildNodeList(mDocument, null, nodeList, false);
        } else {
            getChildNodeList(mDocument, nodeName[0], nodeList, false);
        }
        return;
    }

    public void getChildNodeAtLevel(Node node, List<Node> nodeList, int level, String... nodeName) {
        if (!mIsParseSucceed) {
            return;
        }

        if (nodeName.length == 0) {
            getChildNodeList(node, null, nodeList, false, level);
        } else {
            getChildNodeList(node, nodeName[0], nodeList, false, level);
        }
        return;
    }

    public void getChildNodeAtLevel(List<Node> nodeList, int level, String... nodeName) {
        if (!mIsParseSucceed) {
            return;
        }

        if (nodeName.length == 0) {
            getChildNodeList(mDocument, null, nodeList, false, level);
        } else {
            getChildNodeList(mDocument, nodeName[0], nodeList, false, level);
        }
        return;
    }

    public void getLeafNode(Node node, List<Node> nodeList, String... nodeName) {
        if (!mIsParseSucceed) {
            return;
        }

        if (nodeName.length == 0) {
            getChildNodeList(node, null, nodeList, true);
        } else {
            getChildNodeList(node, nodeName[0], nodeList, true);
        }
        return;
    }

    public void getLeafNode(List<Node> nodeList, String... nodeName) {
        if (!mIsParseSucceed) {
            return;
        }

        if (nodeName.length == 0) {
            getChildNodeList(mDocument, null, nodeList, true);
        } else {
            getChildNodeList(mDocument, nodeName[0], nodeList, true);
        }
        return;
    }

    /**
     * get child node of specified node
     * 
     * Parameters: node - root node; NodeName - find the node with NodeName tag, if NodeName is
     * null, all child node will be added nodeList list to which all node found will be added; level
     * - if the first element, level[0], exist, the search will be done by level, otherwise in
     * pre-order;
     * 
     * 
     */

    protected void getChildNodeList(Node node, String nodeName, List<Node> nodeList,
            boolean onlyleaf, int... level) {
        if (node == null && nodeList == null) {
            return;
        }
        if (((((level.length == 0) && node.getNodeName().equalsIgnoreCase(nodeName)) && nodeName != null) && !onlyleaf)
                || (level.length != 0 && level[0] == 0)) {
            if (node.getNodeType() == Node.ELEMENT_NODE
                    && (nodeName == null || node.getNodeName().equalsIgnoreCase(nodeName))) {
                nodeList.add(node);
            }
            if ((level.length != 0 && level[0] == 0)) {
                return;
            }

        }
        // else { ???
        Node childNode = node.getFirstChild();
        Node siblingNode = childNode;
        if (childNode != null) {

            if (childNode.getNodeType() == Node.ELEMENT_NODE) {
                if (nodeName == null && level.length == 0) {
                    nodeList.add(childNode);
                }
                if (level.length == 0) {
                    getChildNodeList(siblingNode, nodeName, nodeList, onlyleaf);

                } else {
                    getChildNodeList(siblingNode, nodeName, nodeList, onlyleaf, (level[0] - 1));
                }
            }
        }
        boolean isLeaf = true;
        while (siblingNode != null) {
            try {
                siblingNode = siblingNode.getNextSibling();
            } catch (IndexOutOfBoundsException e) {
                siblingNode = null;
            }
            // see node as a leaf
            // Here if a comment node inserted before a text node, the text node
            // will not be add into list as leaf node!!!
            if (siblingNode == null && childNode.getNodeType() == Node.TEXT_NODE) {
                if (onlyleaf && isLeaf) {
                    if (nodeName == null || node.getNodeName().equalsIgnoreCase(nodeName)) {
                        nodeList.add(node);
                    }
                }
            } else {
                isLeaf = false;
            }

            if (siblingNode != null && siblingNode.getNodeType() == Node.ELEMENT_NODE) {
                if (nodeName == null && level.length == 0) {
                    nodeList.add(siblingNode);
                }

                if (level.length == 0) {
                    getChildNodeList(siblingNode, nodeName, nodeList, onlyleaf);

                } else {
                    getChildNodeList(siblingNode, nodeName, nodeList, onlyleaf, (level[0] - 1));
                }
            }
            // }

        }
    }

    public Node getNodeByTreeUri(String treeUri) {
        if (TextUtils.isEmpty(treeUri)) {
            return null;
        }
        String[] strArray = treeUri.split("/");
        int length = strArray.length;

        List<Node> nodeList = new ArrayList<Node>();
        getChildNodeAtLevel(nodeList, 1);

        Node rootNode = nodeList.get(0);

        for (int level = 0; level < length; ++level) {

            boolean isFind = false;
            nodeList = new ArrayList<Node>();
            getChildNodeAtLevel(rootNode, nodeList, 1);
            Log.i(TAG.XML, "rootNode name is " + getDmTreeNodeName(rootNode));

            for (Node node : nodeList) {
                String nodeName = getDmTreeNodeName(node);
                if (strArray[level].equals(nodeName)) {
                    isFind = true;
                    rootNode = node;
                    if (level == length - 1) {
                        Log.i(TAG.XML, "URI name " + strArray[level] + ", nodeName: " + nodeName);
                        return node;
                    }
                    continue;
                }
            }

            if (!isFind) {
                Log.i(TAG.XML, "URI error, please check!");
                return null;
            }
        }
        return null;
    }

    public String getValueByTreeUri(String treeUri) {
        Node node = getNodeByTreeUri(treeUri);
        return getDmTreeLeafVale(node);
    }

    private String getDmTreeNodeName(Node node) {
        if (node == null) {
            return null;
        }
        List<Node> nodeList = new ArrayList<Node>();
        getLeafNode(node, nodeList, DmConst.NodeName.NAME);
        if (nodeList != null && nodeList.size() > 0) {
            return nodeList.get(0).getFirstChild().getNodeValue();
        } else {
            return null;
        }
    }

    private String getDmTreeLeafVale(Node node) {
        if (node == null) {
            return null;
        }
        List<Node> nodeList = new ArrayList<Node>();
        getLeafNode(node, nodeList, DmConst.NodeName.VALUE);
//        Log.i(TAG.XML, "[getDmTreeLeafVale] value node size is " + nodeList.size());
        if (nodeList != null && nodeList.size() > 0) {
            return nodeList.get(0).getFirstChild().getNodeValue();
        } else {
            return null;
        }
    }

}
