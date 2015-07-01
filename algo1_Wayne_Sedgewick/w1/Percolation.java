public class Percolation {

	private WeightedQuickUnionUF unionData;
	private WeightedQuickUnionUF unionDataWithoutBottomVirt;

	private boolean[][] isOpenData;

	private final int ID_TOP_VIRTUAL;
	private final int ID_BOTTOM_VIRTUAL;
	private final int N;

	public Percolation(int n)			   // create N-by-N grid, with all sites blocked
	{
		if (n <= 0)
			throw new IllegalArgumentException();
		N = n;
		unionData = new WeightedQuickUnionUF(N*N+2);
		unionDataWithoutBottomVirt = new WeightedQuickUnionUF(N*N+1);
		ID_TOP_VIRTUAL = N*N;
		ID_BOTTOM_VIRTUAL = N*N+1;
		isOpenData = new boolean[N][N];
		for (int i = 0; i < N; ++i) {
			for (int j = 0; j < N; ++j) {
				isOpenData[i][j] = false;
			}
		}
	}

	private void checkUserIndex(int i, int j)
	{
		if (i < 1 || i > N || j < 1 || j > N)
			throw new IndexOutOfBoundsException();
	}

	private void doUnion(int id1, int id2)
	{
		unionData.union(id1, id2);
		if (id1!=ID_BOTTOM_VIRTUAL && id2!=ID_BOTTOM_VIRTUAL) {
			unionDataWithoutBottomVirt.union(id1, id2);
		}
	}

	public void open(int iUser, int jUser)		  // open site (row i, column j) if it is not open already
	{
		if (isOpen(iUser, jUser))
			return;

		final int i = iUser - 1;
		final int j = jUser - 1;
		final int id = i*N + j;

		// top
		if (i == 0) {
			doUnion(id, ID_TOP_VIRTUAL);
		} else if (isOpen(iUser-1, jUser)) {
			int idTop = (i-1)*N + j;
			doUnion(id, idTop);
		}

		// bottom
		if (i == N-1) {
			doUnion(id, ID_BOTTOM_VIRTUAL);
		} else if (isOpen(iUser+1, jUser)) {
			int idBottom = (i+1)*N + j;
			doUnion(id, idBottom);
		}

		// left
		if (j != 0 && isOpen(iUser, jUser-1)) {
			int idLeft = i*N + (j-1);
			doUnion(id, idLeft);
		}

		// right
		if (j != N-1 && isOpen(iUser, jUser+1)) {
			int idRight = i*N + (j+1);
			doUnion(id, idRight);
		}

		isOpenData[i][j] = true;
	}

	public boolean isOpen(int i, int j)	 // is site (row i, column j) open?
	{
		checkUserIndex(i, j);
		return isOpenData[i-1][j-1];
	}

	public boolean isFull(int iUser, int jUser)	 // is site (row i, column j) full?
	{
		if (!isOpen(iUser, jUser))
			return false;
		final int i = iUser-1;
		final int j = jUser-1;
		final int id = i*N + j;
		return unionDataWithoutBottomVirt.connected(ID_TOP_VIRTUAL, id);
	}

	public boolean percolates()			 // does the system percolate?
	{
		return unionData.connected(ID_TOP_VIRTUAL, ID_BOTTOM_VIRTUAL);
	}

	private void printDbg() {
		for (int i=1; i<=N; ++i) {
			for (int j=1; j<=N; ++j) {
				final char sym = isFull(i,j) ? '*' : (isOpen(i,j) ? ' ' : '#');
				System.out.print(sym);
			}
			System.out.println();
		}
		System.out.println();
	}

	public static void main(String[] args)   // test client (optional)
	{
		Percolation p = new Percolation(3);
		p.open(3,1);
		p.open(3,3);
		p.open(2,3);
		p.printDbg();
		p.open(2,1);
		p.open(1,1);
		p.printDbg();
	}
}
