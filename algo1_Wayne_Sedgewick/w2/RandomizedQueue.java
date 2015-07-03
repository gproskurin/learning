import java.util.*;

public class RandomizedQueue<Item> implements Iterable<Item> {
	private Item[] data = null;
	private int sz = 0;

	public RandomizedQueue()                 // construct an empty randomized queue
	{
	}

	public boolean isEmpty()                 // is the queue empty?
	{
		return sz==0;
	}

	public int size()                        // return the number of items on the queue
	{
		return sz;
	}

	public void enqueue(Item item)           // add the item
	{
		if (item==null)
			throw new NullPointerException();

		if (data==null)
			resize(1);
		else if (sz==data.length)
			resize(sz*2);
		data[sz] = item;
		++sz;
	}

	public Item dequeue()                    // remove and return a random item
	{
		if (isEmpty())
			throw new NoSuchElementException();

		final int idx = StdRandom.uniform(sz);
		Item item = data[idx];

		--sz;
		data[idx] = data[sz];
		data[sz] = null;

		try_shrink();

		return item;
	}

	public Item sample()                     // return (but do not remove) a random item
	{
		if (isEmpty())
			throw new NoSuchElementException();
		return data[StdRandom.uniform(sz)];
	}

	private class Iter implements Iterator<Item> {
		private final int[] order;
		private int cur = 0;

		public Iter() {
			order = new int[size()];
			for (int i=0; i<order.length; ++i) {
				order[i] = i;
			}
			StdRandom.shuffle(order);
		}

		public boolean hasNext() { return cur<order.length; }
		public Item next() {
			if (!hasNext())
				throw new NoSuchElementException();
			return data[order[cur++]];
		}
		public void remove() { throw new UnsupportedOperationException(); }
	}

	public Iterator<Item> iterator()         // return an independent iterator over items in random order
	{
		return new Iter();
	}

	public static void main(String[] args)   // unit testing
	{
	}

	private void resize(int new_cap)
	{
		Item[] new_data = (Item[]) new Object[new_cap];
		if (data!=null) {
			for (int i=0; i<sz; ++i) {
				new_data[i] = data[i];
			}
		}
		data = new_data;
	}

	private void try_shrink()
	{
		if (size() > data.length/4)
			return;
		final int new_cap = size()==0 ? 1 : size()*2;
		assert new_cap >= size();
		resize(new_cap);
	}
}
