/**
 * Matrix Face Editor - Matrix painting app for Tank Plant's faces
 */

import { useState, useCallback, useEffect, useMemo, FC, memo } from 'react';

const WIDTH = 16;
const HEIGHT = 9;
const PIXEL_COUNT = WIDTH * HEIGHT;

// Pixel Component
type PixelProps = {
  color: string;
  index: number;
  onMouseDown: (index: number) => void;
  onMouseEnter: (index: number) => void;
  onTouchStart: (index: number) => void;
};

const Pixel: FC<PixelProps> = memo(({ color, index, onMouseDown, onMouseEnter, onTouchStart }) => (
  <div
    className='border-gray-600 h-30 w-30 cursor-pointer border'
    style={{ backgroundColor: color }}
    onMouseDown={() => onMouseDown(index)}
    onMouseEnter={() => onMouseEnter(index)}
    onTouchStart={(e) => {
      e.preventDefault();
      onTouchStart(index);
    }}
    role='tree'
    tabIndex={0}
  />
));

// Matrix Component
type MatrixProps = {
  matrixData: number[];
  getBrightnessColor: (brightness: number) => string;
  onPixelMouseDown: (index: number) => void;
  onPixelMouseEnter: (index: number) => void;
  onPixelTouchStart: (index: number) => void;
};

const Matrix: FC<MatrixProps> = ({ matrixData, getBrightnessColor, onPixelMouseDown, onPixelMouseEnter, onPixelTouchStart }) => (
  <div className='mx-auto grid max-w-[500px] grid-cols-16 gap-2 rounded-md bg-gray p-2'>
    {matrixData.map((brightness, index) => (
      <Pixel
        key={index}
        index={index}
        color={getBrightnessColor(brightness)}
        onMouseDown={onPixelMouseDown}
        onMouseEnter={onPixelMouseEnter}
        onTouchStart={onPixelTouchStart}
      />
    ))}
  </div>
);

// Tools Panel Component
type ToolsPanelProps = {
  brightness: number;
  setBrightness: (value: number) => void;
  colorMode: string;
  setColorMode: (mode: string) => void;
  brightnessPreviewColor: string;
  onClear: () => void;
  onFill: () => void;
};

const ToolsPanel: FC<ToolsPanelProps> = ({ brightness, setBrightness, colorMode, setColorMode, brightnessPreviewColor, onClear, onFill }) => (
  <div className='my-5 flex w-full flex-col gap-4 rounded-md bg-darkGray p-5 text-white'>
    <h3 className='mb-2 font-martian-mono text-lg font-bold text-white'>Tools</h3>
    <div className='flex max-w-[450px] justify-between'>
      <div className='flex flex-col gap-2'>
        <label htmlFor='brightness'>Brightness ({brightness})</label>
        <input
          type='range'
          id='brightness'
          min='0'
          max='255'
          value={brightness}
          onChange={(e) => setBrightness(Number(e.target.value))}
          className='max-w-[255px]'
        />
        <div className='h-12 max-w-[255px] border border-gray' style={{ backgroundColor: brightnessPreviewColor }} />
      </div>
      <div className='flex flex-col gap-2'>
        <label htmlFor='color-mode'>Color Mode</label>
        <select
          id='color-mode'
          value={colorMode}
          onChange={(e) => setColorMode(e.target.value)}
          className='w-full rounded-md border border-gray bg-gray p-2 text-white'
        >
          <option value='gray'>Grayscale</option>
          <option value='red'>Red</option>
          <option value='green'>Green</option>
          <option value='blue'>Blue</option>
        </select>
      </div>
      <div className='flex flex-col gap-5'>
        <button type='button' onClick={onClear} className='border-red hover:bg-red rounded border px-4 py-2 font-bold text-white'>
          Clear Matrix
        </button>
        <button style={{ backgroundColor: colorMode }} type='button' onClick={onFill} className='boder-white rounded border px-4 py-2 font-bold text-white'>
          Fill Matrix
        </button>
      </div>
    </div>
  </div>
);

// Color Palette Component
type ColorPaletteProps = {
  paletteColors: number[];
  setPaletteColor: (index: number, color: number) => void;
  selectPaletteColor: (color: number) => void;
  getBrightnessColor: (brightness: number) => string;
  currentBrightness: number;
};

const ColorPalette: FC<ColorPaletteProps> = ({ paletteColors, setPaletteColor, selectPaletteColor, getBrightnessColor, currentBrightness }) => (
  <div className='w-full py-10 pt-5'>
    <h3 className='mb-2 font-martian-mono text-lg font-bold text-white'>Color Palette</h3>
    <div className='flex flex-wrap gap-3'>
      {paletteColors.map((color, index) => (
        <div key={index} className='flex flex-col items-center gap-2'>
          <div
            className='border-gray-600 h-14 w-14 cursor-pointer border-2'
            style={{ backgroundColor: getBrightnessColor(color) }}
            onClick={() => selectPaletteColor(color)}
            onKeyUp={() => selectPaletteColor(color)}
            tabIndex={0}
            role='button'
          />
          <button
            type='button'
            onClick={() => setPaletteColor(index, currentBrightness)}
            className='bg-gray-600 hover:bg-gray-700 rounded px-3 py-1 text-xs text-white'
          >
            Set
          </button>
        </div>
      ))}
    </div>
  </div>
);

// Export Controls Component
type ExportControlsProps = {
  matrixData: number[];
  onLoad: (data: number[]) => void;
};

const ExportControls: FC<ExportControlsProps> = ({ matrixData, onLoad }) => {
  const [exportText, setExportText] = useState('');
  const [copyStatus, setCopyStatus] = useState('Copy');
  const [loadStatus, setLoadStatus] = useState('Load');

  const generateExportText = () => {
    const header = `constexpr uint8_t matrixData[${PIXEL_COUNT}] PROGMEM = {\n`;
    const footer = '\n};';

    const rows = Array.from({ length: HEIGHT }, (_, rowIndex) => matrixData.slice(rowIndex * WIDTH, rowIndex * WIDTH + WIDTH));

    const formattedRows = rows.map((row, i) => ` ${row.map((value) => String(value).padStart(3, ' ')).join(', ')}${i < HEIGHT - 1 ? ',' : ''}`);

    const body = formattedRows.join('\n');
    setExportText(header + body + footer);
  };

  const handleCopy = () => {
    if (!exportText) return;
    navigator.clipboard.writeText(exportText).then(() => {
      setCopyStatus('Copied!');
      setTimeout(() => setCopyStatus('Copy'), 2000);
    });
  };

  const handleLoad = () => {
    try {
      const matches = exportText.match(/\{([^}]+)\}/);
      if (!matches?.[1]) throw new Error('Could not find array data in {}.');

      const numbers = matches[1].split(',').map((n) => parseInt(n.trim(), 10));
      if (numbers.some(isNaN) || numbers.length !== 144) {
        throw new Error('Array must contain exactly 144 valid numbers.');
      }
      onLoad(numbers);
      setLoadStatus('Loaded!');
      setTimeout(() => setLoadStatus('Load'), 2000);
    } catch (error) {
      console.log(error instanceof Error ? error.message : 'Invalid array format.');
    }
  };

  return (
    <div className='max-w-2xl mt-6 w-full'>
      <h3 className='mb-2 font-martian-mono text-lg font-bold text-white'>Export Matrix</h3>
      <div className='mb-4 flex gap-4'>
        <button type='button' onClick={generateExportText} className='rounded bg-cl-purple px-4 py-2 font-bold text-white hover:bg-cl-purple'>
          Export as uint8_t Array
        </button>
        <button
          type='button'
          onClick={handleCopy}
          className={`rounded px-4 py-2 font-bold ${copyStatus === 'Copied!' ? 'bg-green' : 'bg-gray hover:bg-gray'} text-white`}
        >
          {copyStatus}
        </button>
        <button
          type='button'
          onClick={handleLoad}
          className={`rounded px-4 py-2 font-bold ${loadStatus === 'Loaded!' ? 'bg-green' : 'bg-gray hover:bg-gray'} text-white`}
        >
          {loadStatus}
        </button>
      </div>
      <textarea
        value={exportText}
        onChange={(e) => setExportText(e.target.value)}
        className='mt-8 min-h-[200px] w-full overflow-auto whitespace-pre rounded-md border border-gray bg-gray p-4 font-martian-mono text-[10px] text-white'
        placeholder='Exported array will appear here...'
      />
    </div>
  );
};

// Main Matrix Face Editor Component
const MatrixFaceEditor = () => {
  const [matrixData, setMatrixData] = useState<number[]>(() => Array(PIXEL_COUNT).fill(0));
  const [paletteColors, setPaletteColors] = useState<number[]>([0, 5, 20, 45, 80, 130, 190, 255]);
  const [brightness, setBrightness] = useState(128);
  const [colorMode, setColorMode] = useState('red');
  const [isDrawing, setIsDrawing] = useState(false);

  const getBrightnessColor = useCallback(
    (b: number): string => {
      const value = Math.min(255, Math.max(0, b));
      switch (colorMode) {
        case 'red':
          return `rgb(${value}, 0, 0)`;
        case 'green':
          return `rgb(0, ${value}, 0)`;
        case 'blue':
          return `rgb(0, 0, ${value})`;
        default:
          return `rgb(${value}, ${value}, ${value})`;
      }
    },
    [colorMode]
  );

  const brightnessPreviewColor = useMemo(() => getBrightnessColor(brightness), [brightness, getBrightnessColor]);

  const handleSetMatrixData = useCallback((index: number, value: number) => {
    setMatrixData((prevData) => {
      const newData = [...prevData];
      newData[index] = value;
      return newData;
    });
  }, []);

  const handlePixelMouseDown = useCallback(
    (index: number) => {
      setIsDrawing(true);
      handleSetMatrixData(index, brightness);
    },
    [brightness, handleSetMatrixData]
  );

  const handlePixelMouseEnter = useCallback(
    (index: number) => {
      if (isDrawing) {
        handleSetMatrixData(index, brightness);
      }
    },
    [isDrawing, brightness, handleSetMatrixData]
  );

  const handlePixelTouchStart = useCallback(
    (index: number) => {
      setIsDrawing(true);
      handleSetMatrixData(index, brightness);
    },
    [brightness, handleSetMatrixData]
  );

  const handleClearMatrix = () => setMatrixData(Array(PIXEL_COUNT).fill(0));
  const handleFillMatrix = () => setMatrixData(Array(PIXEL_COUNT).fill(brightness));

  const handleSetPaletteColor = (index: number, color: number) => {
    setPaletteColors((prev) => {
      const newColors = [...prev];
      newColors[index] = color;
      return newColors;
    });
  };

  const handleSelectPaletteColor = (color: number) => setBrightness(color);

  useEffect(() => {
    const stopDrawing = () => setIsDrawing(false);

    window.addEventListener('mouseup', stopDrawing);
    window.addEventListener('touchend', stopDrawing);
    window.addEventListener('touchcancel', stopDrawing);

    return () => {
      window.removeEventListener('mouseup', stopDrawing);
      window.removeEventListener('touchend', stopDrawing);
      window.removeEventListener('touchcancel', stopDrawing);
    };
  }, []);

  return (
    <div className='flex flex-col items-center bg-darkGray px-15 py-15 text-gray'>
      <h2 className='mb-8 font-martian-mono text-xl font-bold text-white'>16x9 LED Matrix Editor</h2>

      <div className='flex w-full flex-col items-start gap-10'>
        <div className='w-full'>
          <Matrix
            matrixData={matrixData}
            getBrightnessColor={getBrightnessColor}
            onPixelMouseDown={handlePixelMouseDown}
            onPixelMouseEnter={handlePixelMouseEnter}
            onPixelTouchStart={handlePixelTouchStart}
          />
        </div>
        <div className='w-full'>
          <ToolsPanel
            brightness={brightness}
            setBrightness={setBrightness}
            colorMode={colorMode}
            setColorMode={setColorMode}
            brightnessPreviewColor={brightnessPreviewColor}
            onClear={handleClearMatrix}
            onFill={handleFillMatrix}
          />
        </div>
      </div>

      <ColorPalette
        paletteColors={paletteColors}
        setPaletteColor={handleSetPaletteColor}
        selectPaletteColor={handleSelectPaletteColor}
        getBrightnessColor={getBrightnessColor}
        currentBrightness={brightness}
      />

      <ExportControls matrixData={matrixData} onLoad={setMatrixData} />
    </div>
  );
};

export default MatrixFaceEditor;
