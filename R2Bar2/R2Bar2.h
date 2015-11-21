#pragma once

#include <windows.h>
#include <vector>

namespace R2Bar2
{
	// =============
	//   Constants
	// =============

	// Barcode results
	const UINT32 BARCODE_TOO_NARROW = 1;
	const UINT32 BARCODE_NARROW = 2;
	const UINT32 BARCODE_BETWEEN = 4;
	const UINT32 BARCODE_WIDE = 8;
	const UINT32 BARCODE_TOO_WIDE = 16;

	// Symbol ids
	const UINT32 BARCODE_START[6] = {
		BARCODE_NARROW | BARCODE_BETWEEN | BARCODE_WIDE | BARCODE_TOO_WIDE,
		BARCODE_NARROW,
		BARCODE_NARROW,
		BARCODE_NARROW,
		BARCODE_NARROW,
		BARCODE_NARROW
	};
	const UINT32 BARCODE_END[5] = {
		BARCODE_NARROW,
		BARCODE_NARROW,
		BARCODE_WIDE,
		BARCODE_NARROW,
		BARCODE_NARROW | BARCODE_BETWEEN | BARCODE_WIDE | BARCODE_TOO_WIDE
	};
	const UINT32 BARCODE_DIGITS[10][5] = {
		// 0 NNNWW
		{
			BARCODE_NARROW,
			BARCODE_NARROW,
			BARCODE_NARROW,
			BARCODE_WIDE,
			BARCODE_WIDE
		},
		// 1 NNWNW
		{
			BARCODE_NARROW,
			BARCODE_NARROW,
			BARCODE_WIDE,
			BARCODE_NARROW,
			BARCODE_WIDE
		},
		// 2 NNWWN
		{
			BARCODE_NARROW,
			BARCODE_NARROW,
			BARCODE_WIDE,
			BARCODE_WIDE,
			BARCODE_NARROW
		},
		// 3 NWNNW
		{
			BARCODE_NARROW,
			BARCODE_WIDE,
			BARCODE_NARROW,
			BARCODE_NARROW,
			BARCODE_WIDE
		},
		// 4 NWNWN
		{
			BARCODE_NARROW,
			BARCODE_WIDE,
			BARCODE_NARROW,
			BARCODE_WIDE,
			BARCODE_NARROW
		},
		// 5 NWWNN
		{
			BARCODE_NARROW,
			BARCODE_WIDE,
			BARCODE_WIDE,
			BARCODE_NARROW,
			BARCODE_NARROW
		},
		// 6 WNNNW
		{
			BARCODE_WIDE,
			BARCODE_NARROW,
			BARCODE_NARROW,
			BARCODE_NARROW,
			BARCODE_WIDE
		},
		// 7 WNNWN
		{
			BARCODE_WIDE,
			BARCODE_NARROW,
			BARCODE_NARROW,
			BARCODE_WIDE,
			BARCODE_NARROW
		},
		// 8 WNWNN
		{
			BARCODE_WIDE,
			BARCODE_NARROW,
			BARCODE_WIDE,
			BARCODE_NARROW,
			BARCODE_NARROW
		},
		// 9 WWNNN
		{
			BARCODE_WIDE,
			BARCODE_WIDE,
			BARCODE_NARROW,
			BARCODE_NARROW,
			BARCODE_NARROW
		}
	};

	// =======================
	//   Structs and Classes
	// =======================
	struct Result
	{
		UINT32 value, digits, start, end;
	};

	struct Error
	{
		FLOAT width, error, minN, maxN, minW, maxW;
	};

	// =============
	//   Utilities
	// =============

	// Convert UINT32 ARGB image to ARGB monocolor image
	void argbToMono(UINT32 length, const UINT32 * in, UINT32 * out)
	{
		constexpr UINT32 black = 0xFF000000;
		constexpr UINT32 white = 0xFFFFFFFF;

		while (length--)
		{
			UINT32 c = *in;
			UINT32 r = (c >> 16) & 0xFF;
			UINT32 g = (c >> 8) & 0xFF;
			UINT32 b = (c >> 0) & 0xFF;

			if (r + g + b > 127 * 3)
			{
				*out = white;
			}
			else
			{
				*out = black;
			}
		}
	}

	// Convert 1 dimentional UINT32 ARGB image to monocolor run-length vector (first element is black)
	void argbToMonoRLE(UINT32 length, const UINT32 * in, std::vector<UINT32> & out)
	{
		constexpr UINT32 black = 0xFF000000;
		constexpr UINT32 white = 0xFFFFFFFF;

		UINT32 prev = black;
		UINT32 count = 0;

		while (length--)
		{
			UINT32 c = *in;
			in++;
			UINT32 r = (c >> 16) & 0xFF;
			UINT32 g = (c >> 8) & 0xFF;
			UINT32 b = (c >> 0) & 0xFF;

			if (r + g + b > 127 * 3)
			{
				if (prev == white)
				{
					count++;
				}
				else
				{
					out.push_back(count);
					count = 1;
					prev = white;
				}
			}
			else
			{
				if (prev == black)
				{
					count++;
				}
				else
				{
					out.push_back(count);
					count = 1;
					prev = black;
				}
			}
		}

		out.push_back(count);
	}

	// Transpose a matrix
	void transpose(UINT32 width, UINT32 height, const UINT32 * in, UINT32 * out)
	{
		for (UINT32 i = 0; i < height; i++)
		{
			for (UINT32 j = 0; j < width; j++)
			{
				out[i + j * height] = in[i * width + j];
			}
		}
	}

	// Create result struct
	void setResult(Result & result, UINT32 start, UINT32 end, UINT32 value, UINT32 digits)
	{
		result.start = start;
		result.end = end;
		result.value = value;
		result.digits = digits;
	}

	// Create and compute error bounds for a certain bar width
	void setError(Error & error, UINT32 width)
	{
		error.width = (FLOAT) width;
		error.error = (width + 1.f) / 4.f;
		error.minN = width - error.error;
		error.maxN = width + error.error;
		error.minW = error.minN * 2.f;
		error.maxW = error.maxN * 2.f;
	}

	// Matches a barcode width with error bounds
	UINT32 matchBarcode(Error err, UINT32 w)
	{
		if (w < err.minN)
		{
			return BARCODE_TOO_NARROW;
		}
		else if (w <= err.maxN)
		{
			return BARCODE_NARROW;
		}
		else if (w < err.minW)
		{
			return BARCODE_BETWEEN;
		}
		else if (w <= err.maxW)
		{
			return BARCODE_WIDE;
		}
		else
		{
			return BARCODE_TOO_WIDE;
		}
	}

	// ===========
	//   Parsers
	// ===========

	// Attempt to parse a barcode sequence at a certain index in the run-length vector.
	// Returns number of runs read, or -1 if not possible.
	INT32 parseRun(const std::vector<UINT32> & runs, INT32 index, Error err, const UINT32 * symbol, const UINT32 symbolLength)
	{
		for (UINT32 i = 0; i < symbolLength; i++)
		{
			// If we go out of bounds, we are done
			if (index + i >= (INT32) runs.size())
			{
				return -1;
			}

			// Otherwise ensure that the next index is of appropriate width
			INT32 nextWidth = matchBarcode(err, runs[index + i]);
			if ((symbol[i] & nextWidth) == 0)
			{
			  // Widths do not match
				return -1;
			}
		}

		return symbolLength;
	}

	// Attempt to parse the start of a barcode at a certain index in the run-length vector.
	// Returns number of runs read, or -1 if not possible.
	INT32 parseStart(const std::vector<UINT32> & runs, INT32 index, Error err)
	{
		return parseRun(runs, index, err, BARCODE_START, 6);
	}

	// Attempt to parse the end of a barcode at a certain index in the run-length vector.
	// Returns number of runs read, or -1 if not possible.
	INT32 parseEnd(const std::vector<UINT32> & runs, INT32 index, Error err)
	{
		return parseRun(runs, index, err, BARCODE_END, 5);
	}

	// Attempt to parse a digit of a barcode at a certain index in the run-length vector.
	// Returns number of runs read, or -1 if not possible.
	INT32 parseDigit(const std::vector<UINT32> & runs, UINT32 index, Error err, UINT32 digit)
	{
		return parseRun(runs, index, err, BARCODE_DIGITS[digit], 5);
	}

	// Attempt to parse a barcode at a certain index in the run-length vector.
	// Returns number of runs read and fills in result struct, or -1 if not possible.
	INT32 parseBarcode(const std::vector<UINT32> & runs, INT32 start, Result & result)
	{
		// Try all widths down to 0
		for (int width = runs[start]; width > 0; width--)
		{
			INT32 index = start;
			// Get error bounds
			Error e = {};
			setError(e, width);

			// Try to read the identifier
			INT32 read;

			read = parseStart(runs, index, e);
			if (read >= 0)
			{
				// Read digits
				index = index + read;

				INT32 value = 0;
				INT32 count = 0;
				BOOL done = false;
				while (!done)
				{
					done = true;

					// First try to read digits
					for (INT32 i = 0; i < 10 && done; i++)
					{
						read = parseDigit(runs, index, e, i);
						if (read >= 0)
						{
							index = index + read;
							done = false;
							value = value * 10 + i;
							count++;
						}
					}

					if (done)
					{ // No digits have been found
						read = parseEnd(runs, index, e);
						if (read >= 0)
						{
							// Completed barcode found!
							index = index + read;
							setResult(result, start, index, value, count);
							return index - start;
						}
					}
				}

				// By this point, no end has been found,
				// and no digits have been matched
			}
		}
		return -1;
	}

	// ===========
	//   Decoder
	// ===========

	// Read a barcode from a run-length vector, starting at a certain index.
	// Return index of found barcode and fills in result struct, or -1 if none found.
	INT32 decode(const std::vector<UINT32> & runs, INT32 index, Result & result)
	{
		setResult(result, -1, -1, -1, -1);

		// Loop through every black bar
		INT32 read;
		for (INT32 i = index, l = runs.size(); i < l; i += 2)
		{
			if ((read = parseBarcode(runs, i, result)) >= 0)
			{
				return index + read;
			}
		}

		return -1;
	}

	// ===========
	//   Encoder
	// ===========

	// TODO
};
