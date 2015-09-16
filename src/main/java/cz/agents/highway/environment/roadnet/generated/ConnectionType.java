//
// This file was generated by the JavaTM Architecture for XML Binding(JAXB) Reference Implementation, vJAXB 2.1.10 
// See <a href="http://java.sun.com/xml/jaxb">http://java.sun.com/xml/jaxb</a> 
// Any modifications to this file will be lost upon recompilation of the source schema. 
// Generated on: 2015.09.15 at 11:39:45 AM CEST 
//


package cz.agents.highway.environment.roadnet.generated;

import java.math.BigInteger;
import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;
import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlSchemaType;
import javax.xml.bind.annotation.XmlType;


/**
 * <p>Java class for connectionType complex type.
 * 
 * <p>The following schema fragment specifies the expected content contained within this class.
 * 
 * <pre>
 * &lt;complexType name="connectionType">
 *   &lt;complexContent>
 *     &lt;restriction base="{http://www.w3.org/2001/XMLSchema}anyType">
 *       &lt;attribute name="from" use="required" type="{http://www.w3.org/2001/XMLSchema}string" />
 *       &lt;attribute name="to" use="required" type="{http://www.w3.org/2001/XMLSchema}string" />
 *       &lt;attribute name="fromLane" use="required" type="{http://www.w3.org/2001/XMLSchema}nonNegativeInteger" />
 *       &lt;attribute name="toLane" use="required" type="{http://www.w3.org/2001/XMLSchema}nonNegativeInteger" />
 *       &lt;attribute name="pass" type="{}boolType" />
 *       &lt;attribute name="keepClear" type="{}boolType" />
 *       &lt;attribute name="via" type="{http://www.w3.org/2001/XMLSchema}string" />
 *       &lt;attribute name="tl" type="{http://www.w3.org/2001/XMLSchema}string" />
 *       &lt;attribute name="linkIndex" type="{http://www.w3.org/2001/XMLSchema}nonNegativeInteger" />
 *       &lt;attribute name="dir" use="required">
 *         &lt;simpleType>
 *           &lt;restriction base="{http://www.w3.org/2001/XMLSchema}string">
 *             &lt;enumeration value="s"/>
 *             &lt;enumeration value="t"/>
 *             &lt;enumeration value="T"/>
 *             &lt;enumeration value="l"/>
 *             &lt;enumeration value="r"/>
 *             &lt;enumeration value="L"/>
 *             &lt;enumeration value="R"/>
 *           &lt;/restriction>
 *         &lt;/simpleType>
 *       &lt;/attribute>
 *       &lt;attribute name="state" use="required">
 *         &lt;simpleType>
 *           &lt;restriction base="{http://www.w3.org/2001/XMLSchema}string">
 *             &lt;enumeration value="M"/>
 *             &lt;enumeration value="m"/>
 *             &lt;enumeration value="o"/>
 *             &lt;enumeration value="="/>
 *             &lt;enumeration value="-"/>
 *             &lt;enumeration value="s"/>
 *             &lt;enumeration value="w"/>
 *           &lt;/restriction>
 *         &lt;/simpleType>
 *       &lt;/attribute>
 *     &lt;/restriction>
 *   &lt;/complexContent>
 * &lt;/complexType>
 * </pre>
 * 
 * 
 */
@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "connectionType")
public class ConnectionType {

    @XmlAttribute(required = true)
    protected String from;
    @XmlAttribute(required = true)
    protected String to;
    @XmlAttribute(required = true)
    @XmlSchemaType(name = "nonNegativeInteger")
    protected BigInteger fromLane;
    @XmlAttribute(required = true)
    @XmlSchemaType(name = "nonNegativeInteger")
    protected BigInteger toLane;
    @XmlAttribute
    protected String pass;
    @XmlAttribute
    protected String keepClear;
    @XmlAttribute
    protected String via;
    @XmlAttribute
    protected String tl;
    @XmlAttribute
    @XmlSchemaType(name = "nonNegativeInteger")
    protected BigInteger linkIndex;
    @XmlAttribute(required = true)
    protected String dir;
    @XmlAttribute(required = true)
    protected String state;

    /**
     * Gets the value of the from property.
     * 
     * @return
     *     possible object is
     *     {@link String }
     *     
     */
    public String getFrom() {
        return from;
    }

    /**
     * Sets the value of the from property.
     * 
     * @param value
     *     allowed object is
     *     {@link String }
     *     
     */
    public void setFrom(String value) {
        this.from = value;
    }

    /**
     * Gets the value of the to property.
     * 
     * @return
     *     possible object is
     *     {@link String }
     *     
     */
    public String getTo() {
        return to;
    }

    /**
     * Sets the value of the to property.
     * 
     * @param value
     *     allowed object is
     *     {@link String }
     *     
     */
    public void setTo(String value) {
        this.to = value;
    }

    /**
     * Gets the value of the fromLane property.
     * 
     * @return
     *     possible object is
     *     {@link BigInteger }
     *     
     */
    public BigInteger getFromLane() {
        return fromLane;
    }

    /**
     * Sets the value of the fromLane property.
     * 
     * @param value
     *     allowed object is
     *     {@link BigInteger }
     *     
     */
    public void setFromLane(BigInteger value) {
        this.fromLane = value;
    }

    /**
     * Gets the value of the toLane property.
     * 
     * @return
     *     possible object is
     *     {@link BigInteger }
     *     
     */
    public BigInteger getToLane() {
        return toLane;
    }

    /**
     * Sets the value of the toLane property.
     * 
     * @param value
     *     allowed object is
     *     {@link BigInteger }
     *     
     */
    public void setToLane(BigInteger value) {
        this.toLane = value;
    }

    /**
     * Gets the value of the pass property.
     * 
     * @return
     *     possible object is
     *     {@link String }
     *     
     */
    public String getPass() {
        return pass;
    }

    /**
     * Sets the value of the pass property.
     * 
     * @param value
     *     allowed object is
     *     {@link String }
     *     
     */
    public void setPass(String value) {
        this.pass = value;
    }

    /**
     * Gets the value of the keepClear property.
     * 
     * @return
     *     possible object is
     *     {@link String }
     *     
     */
    public String getKeepClear() {
        return keepClear;
    }

    /**
     * Sets the value of the keepClear property.
     * 
     * @param value
     *     allowed object is
     *     {@link String }
     *     
     */
    public void setKeepClear(String value) {
        this.keepClear = value;
    }

    /**
     * Gets the value of the via property.
     * 
     * @return
     *     possible object is
     *     {@link String }
     *     
     */
    public String getVia() {
        return via;
    }

    /**
     * Sets the value of the via property.
     * 
     * @param value
     *     allowed object is
     *     {@link String }
     *     
     */
    public void setVia(String value) {
        this.via = value;
    }

    /**
     * Gets the value of the tl property.
     * 
     * @return
     *     possible object is
     *     {@link String }
     *     
     */
    public String getTl() {
        return tl;
    }

    /**
     * Sets the value of the tl property.
     * 
     * @param value
     *     allowed object is
     *     {@link String }
     *     
     */
    public void setTl(String value) {
        this.tl = value;
    }

    /**
     * Gets the value of the linkIndex property.
     * 
     * @return
     *     possible object is
     *     {@link BigInteger }
     *     
     */
    public BigInteger getLinkIndex() {
        return linkIndex;
    }

    /**
     * Sets the value of the linkIndex property.
     * 
     * @param value
     *     allowed object is
     *     {@link BigInteger }
     *     
     */
    public void setLinkIndex(BigInteger value) {
        this.linkIndex = value;
    }

    /**
     * Gets the value of the dir property.
     * 
     * @return
     *     possible object is
     *     {@link String }
     *     
     */
    public String getDir() {
        return dir;
    }

    /**
     * Sets the value of the dir property.
     * 
     * @param value
     *     allowed object is
     *     {@link String }
     *     
     */
    public void setDir(String value) {
        this.dir = value;
    }

    /**
     * Gets the value of the state property.
     * 
     * @return
     *     possible object is
     *     {@link String }
     *     
     */
    public String getState() {
        return state;
    }

    /**
     * Sets the value of the state property.
     * 
     * @param value
     *     allowed object is
     *     {@link String }
     *     
     */
    public void setState(String value) {
        this.state = value;
    }

}
