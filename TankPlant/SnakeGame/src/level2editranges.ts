main.cpp =>

editRanges: [
    {
        id: 'WORLD_WRAP',
        range: [76, 1, 76, 22],
        values: [
            '#define WORLD_WRAP',
            '// #define WORLD_WRAP',
        ],
    },
    {
        id: 'HARD_MODE',
        range: [77, 1, 77, 22],
        values: [
            '#define HARD_MODE',
            '// #define HARD_MODE',
        ],
    },
    {
        id: 'DIAGONAL_MODE',
        range: [78, 1, 78, 25],
        values: [
            '// #define DIAGONAL_MODE',
            '#define DIAGONAL_MODE',
        ],
    },
    {
        id: 'JUMPER_SET',
        range: [84, 1, 84, 22],
        values: [
            '// #define JUMPER_SET',
            '#define JUMPER_SET',
        ],
    },
    {
        id: 'SNAKE_STARTING_LEN',
        range: [89, 40, 89, 42],
        validation: (value) => {
            const num = parseInt(value);            
            if (num < 2 || num > 20) {
            return {
                valid: false,
                message: 'Value must be between 2 and 20',
            };
            }
            return { valid: true, message: '' };
        },
    },
    {
        id: 'MAX_SNAKE_LEN',
        range: [90, 35, 90, 38],
        validation: (value) => {
            const num = parseInt(value);            
            if (num < 3 || num > 144) {
            return {
                valid: false,
                message: 'Value must be between 3 and 144',
            };
            }
            return { valid: true, message: '' };
        },
    },
    {
        id: 'DEFAULT_MOVE_INTERVAL',
        range: [91, 44, 91, 48],
        validation: (value) => {
            const num = parseInt(value);            
            if (num < 1 || num > 1000) {
            return {
                valid: false,
                message: 'Value must be between 1 and 1000',
            };
            }
            return { valid: true, message: '' };
        },
    },
    {
        id: 'MIN_MOVE_INTERVAL',
        range: [92, 40, 92, 44],
        validation: (value) => {
            const num = parseInt(value);            
            if (num < 1 || num > 999) {
            return {
                valid: false,
                message: 'Value must be between 1 and 999',
            };
            }
            return { valid: true, message: '' };
        },
    },
    {
        id: 'snakeMoveIntSubtractor',
        range: [93, 35, 93, 37],
        validation: (value) => {
            const num = parseInt(value);            
            if (num < 1 || num > 999) {
            return {
                valid: false,
                message: 'Value must be between 1 and 999',
            };
            }
            return { valid: true, message: '' };
        },
    },
]
