package org.firstinspires.ftc.teamcode.utils;

import java.util.Stack;

/**
 * Implements a stack that can contain a fixed number of values. If the size of the stack would
 * exceed it's maximum size, the oldest element in the stack is removed to make space for the
 * new element.
 * @param <T> the type of stack elements.
 */
public class SizedStack<T> extends Stack<T> {
    private int maxSize;

    /**
     * Initialize the stack so that it contains at most <code>size</code> elements.
     * @param size the maximum capacity of the sized stack.
     */
    public SizedStack(int size) {
        super();
        this.maxSize = size;
    }

    /**
     * Adds an element to the stack. If the capacity of the stack would exceed the maximum allowed
     * number of elements, the oldest element in the stack is first removed.
     * @param object   the item to be pushed onto this stack.
     * @return The object that was added to the stack.
     */
    @Override
    public T push(T object) {
        //If the stack is too big, remove elements until it's the right size.
        while (this.size() >= maxSize) {
            this.remove(0);
        }
        return super.push(object);
    }
}