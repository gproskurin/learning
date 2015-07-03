import java.util.*;

public class Deque<Item> implements Iterable<Item> {
	private class Node {
		Item item;
		Node next;
		Node prev;
		Node(Item i, Node n, Node p) {
			item = i;
			next = n;
			prev = p;
		}
	}

	private Node first = null;
	private Node last = null;
	private int sz = 0;

	public Deque()                           // construct an empty deque
	{
	}

	public boolean isEmpty()                 // is the deque empty?
	{
		sanity_check();
		return sz==0;
	}

	public int size()                        // return the number of items on the deque
	{
		sanity_check();
		return sz;
	}

	public void addFirst(Item item)          // add the item to the front
	{
		if (item==null)
			throw new NullPointerException();
		first = new Node(item, first, null);
		if (last==null) {
			assert sz==0;
			last = first;
		} else {
			first.next.prev = first;
		}
		++sz;
	}

	public void addLast(Item item)           // add the item to the end
	{
		if (item==null)
			throw new NullPointerException();
		last = new Node(item, null, last);
		if (first==null) {
			assert sz==0;
			first = last;
		} else {
			last.prev.next = last;
		}
		++sz;
	}

	public Item removeFirst()                // remove and return the item from the front
	{
		if (isEmpty())
			throw new NoSuchElementException();
		Item i = first.item;
		first = first.next;
		if (first==null) {
			assert sz==1;
			last = null;
		} else {
			first.prev = null;
		}
		--sz;
		return i;
	}

	public Item removeLast()                 // remove and return the item from the end
	{
		if (isEmpty())
			throw new NoSuchElementException();
		Item i = last.item;
		last = last.prev;
		if (last==null) {
			assert sz==1;
			first = null;
		} else {
			last.next = null;
		}
		--sz;
		return i;
	}

	public Iterator<Item> iterator()         // return an iterator over items in order from front to end
	{
		return new Iter();
	}

	private void print() {
		System.out.print("DATA(sz=" + size() + "):");
		for (Item i : this) {
			System.out.print(" " + i);
		}
		System.out.println();
	}

	public static void main(String[] args)   // unit testing
	{
		Deque<Integer> d = new Deque<Integer>();
		assert d.isEmpty();
		assert d.size()==0;

		d.addFirst(4);
		assert !d.isEmpty();
		assert d.size()==1;
		//d.print();

		assert d.removeLast()==4;
		assert d.isEmpty();
		assert d.size()==0;
		//d.print();

		d.addLast(5);
		assert !d.isEmpty();
		assert d.size()==1;
		d.addLast(6);
		assert !d.isEmpty();
		assert d.size()==2;
		d.print();

		assert d.removeFirst()==5;
		assert !d.isEmpty();
		assert d.size()==1;

		assert d.removeFirst()==6;
		assert d.isEmpty();
		assert d.size()==0;

	}

	private class Iter implements Iterator<Item> {
		private Node current = first;
		public boolean hasNext() { return current!=null; }
		public Item next() {
			if (current==null)
				throw new NoSuchElementException();
			Item i = current.item;
			current = current.next;
			return i;
		}
		public void remove() { throw new UnsupportedOperationException(); }
	}

	private void sanity_check() {
		if (first==null || last==null) {
			assert first==last;
			assert sz==0;
		}
		if (first != null) {
			assert first.prev == null;
			assert sz>0;
		}
		if (last != null) {
			assert last.next == null;
			assert sz>0;
		}
	}
}

//public class Deque<E> {
/*
	private int size_ = 0;

	private E[] data_;
	private int data_begin_;
	private int data_end_; // last+1 (as C++ iterator)

	Deque() { clear(); }

	public int size() { return data_end_ - data_begin_; }

	public boolean empty() { return data_begin_ == data_end_; }

	public void push_front(E new_item)
	{
		reserve_front();
		assert data_begin_ > 0;
		--data_begin_;
		data_[data_begin_] = new_item;
	}

	public void push_back(E new_item) {
		reserve_back();
		assert data_end_ < data_.length;
		data_[data_end_] = new_item;
		++data_end_;
	}

	public void pop_front() {
		assert !this.empty();
		data_[data_begin_] = null;
		++data_begin_;
	}

	public void pop_back() {
		assert !this.empty();
		--data_end_;
		data_[data_end_] = null;
	}

	public void clear() {
		data_ = create_array(10);
		data_begin_ = data_.length / 2;
		data_end_ = data_begin_;
	}

	private void reserve_front() {
		if (data_begin_ == 0) {
			final int shift = reserve_size();
			E[] new_data = create_array(shift + data_.length);
			for (int i=data_begin_; i<data_end_; ++i) {
				new_data[i+shift] = data_[i];
			}
			data_ = new_data;
			data_begin_ = shift;
			data_end_ += shift;
		}
	}

	private void reserve_back() {
		if (data_end_ >= data_.length) {
			E[] new_data = create_array(data_.length + reserve_size());
			for (int i=data_begin_; i<data_end_; ++i) {
				new_data[i] = data_[i];
			}
			data_ = new_data;
		}
	}

	private int reserve_size() { return 10; }

	private E[] create_array(int sz) {
		return (E[]) Array.newInstance(data_.getClass().getComponentType(), sz);
	}
}
*/

