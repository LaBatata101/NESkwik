package com.labatata.neskwik;

import android.app.Activity;
import android.content.Intent;
import android.database.Cursor;
import android.net.Uri;
import android.provider.OpenableColumns;

import org.libsdl.app.SDLActivity;

public class NeskwikActivity extends SDLActivity {
    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        super.onActivityResult(requestCode, resultCode, data);
    }

    public String getDisplayNameForUri(String uriText) {
        if (uriText == null || !uriText.startsWith("content://")) {
            return null;
        }

        Uri uri = Uri.parse(uriText);
        Cursor cursor = null;
        try {
            cursor = getContentResolver().query(
                    uri,
                    new String[] { OpenableColumns.DISPLAY_NAME },
                    null,
                    null,
                    null
                );
            cursor.moveToFirst();
            return cursor.getString(cursor.getColumnIndex(OpenableColumns.DISPLAY_NAME));
        } finally {
            if (cursor != null) {
                cursor.close();
            }
        }
    }
}
