import edu.princeton.cs.algs4.StdIn;
import edu.princeton.cs.algs4.StdOut;

public class Subset {
	public static void main(String[] args)
	{
		final int k = Integer.parseInt(args[0]);
		RandomizedQueue<String> rq = new RandomizedQueue<String>();
		while (!StdIn.isEmpty()) {
			String s = StdIn.readString();
			rq.enqueue(s);
		}
		assert k <= rq.size();

		int printed = 0;
		for (String s : rq) {
			if (printed >= k)
				break;
			StdOut.println(s);
			++printed;
		}
	}
}
