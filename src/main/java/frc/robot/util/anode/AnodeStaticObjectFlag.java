package frc.robot.util.anode;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.TYPE)
/**
 * Flags an anode object as statically logged
 * <p>This means you can use it without setting an instance name parameter.</p>
 * <b>NOTE: If you set this on a class with multiple instances, the logged names will be the same, causing the values to be shared, and 
 * potentially causing unintended behavior on construction</b>
 */
public @interface AnodeStaticObjectFlag {}
