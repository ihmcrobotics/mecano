package us.ihmc.mecano.algorithms;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.tools.EuclidCoreIOTools;

public class TablePrinter
{
   public enum Alignment
   {
      LEFT, RIGHT
   }

   private final List<List<Cell>> tableRows = new ArrayList<>();
   private int numberOfColumns = 0;
   private String columnSeparator = "\t";
   private boolean needColumnResizing = false;
   private String doubleFormat = EuclidCoreIOTools.getStringFormat(11, 7);

   public TablePrinter()
   {
   }

   public void setColumnSeparator(String columnSeparator)
   {
      this.columnSeparator = columnSeparator;
   }

   public void setDoubleFormat(String doubleFormat)
   {
      this.doubleFormat = doubleFormat;
   }

   public void setRow(int row, String... text)
   {
      for (int col = 0; col < text.length; col++)
         setCell(row, col, text[col], Alignment.LEFT);
   }

   public void setSubTable(int startRow, int startCol, DMatrixRMaj matrix)
   {
      for (int row = 0; row < matrix.getNumRows(); row++)
      {
         for (int col = 0; col < matrix.getNumCols(); col++)
         {
            setNumeric(row + startRow, col + startCol, matrix.get(row, col));
         }
      }
   }

   public void setNumeric(int row, int col, double value)
   {
      setCell(row, col, String.format(doubleFormat, value), Alignment.RIGHT);
   }

   public void setCell(int row, int col, String value, Alignment alignment)
   {
      while (tableRows.size() <= row)
         tableRows.add(new ArrayList<>());

      List<Cell> tableRow = tableRows.get(row);

      while (tableRow.size() <= col)
         tableRow.add(new Cell());

      tableRow.set(col, new Cell(value, alignment));

      numberOfColumns = Math.max(tableRow.size(), numberOfColumns);
      needColumnResizing = true;
   }

   private void resizeColumns()
   {
      if (!needColumnResizing)
         return;

      int[] columnWidths = new int[numberOfColumns];

      for (int row = 0; row < tableRows.size(); row++)
      {
         List<Cell> tableRow = tableRows.get(row);

         while (tableRow.size() < numberOfColumns)
            tableRow.add(new Cell());

         for (int col = 0; col < numberOfColumns; col++)
         {
            Cell cell = tableRow.get(col);
            cell.trim();
            columnWidths[col] = Math.max(cell.length(), columnWidths[col]);
         }
      }

      for (int row = 0; row < tableRows.size(); row++)
      {
         List<Cell> tableRow = tableRows.get(row);

         for (int col = 0; col < numberOfColumns; col++)
         {
            tableRow.get(col).padToLength(columnWidths[col]);
         }
      }

      needColumnResizing = false;
   }

   @Override
   public String toString()
   {
      resizeColumns();
      return EuclidCoreIOTools.getCollectionString("\n", tableRows, tableRow -> rowToString(tableRow));
   }

   public String rowToString(int rowIndex)
   {
      resizeColumns();
      return rowToString(tableRows.get(rowIndex));
   }

   private String rowToString(List<Cell> tableRow)
   {
      return EuclidCoreIOTools.getCollectionString(columnSeparator, tableRow, Cell::toString);
   }

   private static class Cell
   {
      String text = "";
      Alignment alignment = Alignment.RIGHT;

      public Cell()
      {
      }

      public Cell(String text, Alignment alignment)
      {
         this.text = text;
         this.alignment = alignment;
      }

      public void trim()
      {
         text = text.trim();
      }

      public int length()
      {
         return text.length();
      }

      public void padToLength(int desiredLength)
      {
         if (length() >= desiredLength)
            return;

         if (alignment == Alignment.LEFT)
            text = String.format("%-" + desiredLength + "s", text);
         else
            text = String.format("%" + desiredLength + "s", text);
      }

      @Override
      public String toString()
      {
         return text;
      }
   }
}
